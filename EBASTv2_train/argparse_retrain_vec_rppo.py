"""
Support Vectorized Environment for parallel computation
"""

from pathlib import Path
import sys
import os
import argparse

# =========================
# PATH HELPER
# =========================
ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

# Workaround for OpenMP duplicate runtime
os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

# Important to keep to prevent sb3-contrib importing torch from ARS that causes error
import numpy as np
import torch

from sb3_contrib import RecurrentPPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.vec_env import SubprocVecEnv
from multiprocessing import freeze_support

# Ensure libcosim DLL is found on WINDOWS
# Note: os.add_dll_directory only exists on Windows.
# On Linux/HPC, shared libraries should be handled by LD_LIBRARY_PATH.
if sys.platform.startswith("win"):
    dll_dir = Path(sys.prefix) / "Lib" / "site-packages" / "libcosimpy" / "libcosimc"

    if dll_dir.exists():
        os.add_dll_directory(str(dll_dir))
    else:
        print(f"Warning: libcosim DLL directory not found: {dll_dir}")

from EBASTv2_core.env import EBASTv2Env
from EBASTv2_core.episode_logger import log_training_args
from EBASTv2_core.path_utils import get_re_trained_RL_model_path
from orchestrator.scenario_config import load_spawn_requests_bank_path


def str2bool(value):
    """Parse boolean CLI arguments safely."""
    if isinstance(value, bool):
        return value
    value = value.lower()
    if value in {"yes", "true", "t", "1", "y"}:
        return True
    if value in {"no", "false", "f", "0", "n"}:
        return False
    raise argparse.ArgumentTypeError("Boolean value expected.")

def print_debug(msg, debug):
    if debug:
        print(msg)

def parse_cli_args():
    parser = argparse.ArgumentParser(description="EB-ASTv2 with RecurrentPPO model")

    # Run setup
    parser.add_argument("--model_name", type=str, default="EB-ASTv2_train_rppo", metavar="MODEL_NAME",
                        help="RUN: model/run name used for output folders (default: EB-ASTv2_train_rppo)")
    parser.add_argument("--save_anim_filename", type=str, default="EBASTv2_train_rppo.gif", metavar="SAVE_ANIM_FILENAME",
                        help="RUN: animation filename to save (default: EBASTv2_train_rppo.gif)")
    parser.add_argument("--save_reward_function", type=str2bool, default=True, metavar="SAVE_REWARD_FUNCTION",
                        help="RUN: save the reward function and automatically store it the log (default: True)")

    # RL Environment and Spawn request generation
    parser.add_argument("--use_fmpy", type=str2bool, default=True, metavar="USE_FMPY",
                        help="ENV: enable the use of FMPy instead of Libcosimpy orchestrator to enable proper FMU reset (default: True)")
    parser.add_argument("--n_cases", type=int, default=1000, metavar="N_CASES",
                        help="ENV: number of spawn request cases to generate/collect (default: 1000)")
    parser.add_argument("--training_case_ratio", type=float, default=0.9, metavar="TRAINING_CASE_RATIO",
                        help="ENV: ratio of cases used for training in RL env (default: 0.9)")
    parser.add_argument("--overwrite_spawn_bank", type=str2bool, default=False, metavar="OVERWRITE_SPAWN_BANK",
                        help="ENV: whether to overwrite existing spawn request bank (default: False)")
    parser.add_argument("--debug", type=str2bool, default=False, metavar="DEBUG",
                        help="ENV: toogle on env debug mode (default: False)")
    
    # Environment vectorization
    parser.add_argument("--n_envs", type=int, default=64, metavar="N_ENVS",
                        help="VEC_ENV: The number of environment instances for computating parallelization (default: 64)")

    # RecurrentPPO core
    parser.add_argument("--total_timesteps", type=int, default=2_000_000, metavar="TOTAL_TIMESTEPS",
                        help="AST: total model training timesteps. Ideally bigger than n_steps (default: 10_000_000)")
    parser.add_argument("--tensorboard_log", type=str2bool, default=True, metavar="TENSORBOARD_LOG",
                        help="AST: enable tensorboard logging to training folder (default: True)")
    parser.add_argument("--verbose", type=int, default=0, metavar="VERBOSE",
                        help="AST: verbosity level (default: 0)")
    parser.add_argument("--seed", type=int, default=None, metavar="SEED",
                        help="AST: random seed (default: None)")
    parser.add_argument("--device", type=str, default="cuda", metavar="DEVICE",
                        help="AST: device to use, e.g. cpu, cuda, auto (default: cuda)")

    return parser.parse_args()


def main():
    # =========================
    # PATH HELPER
    # =========================
    ROOT = Path(__file__).resolve().parents[1]
    if str(ROOT) not in sys.path:
        sys.path.insert(0, str(ROOT))
    
    # Parse args
    args = parse_cli_args()
    print_debug("[MAIN] Args parsed",
                debug=args.debug)
    
    # Print torch version
    print_debug(f"[MAIN] Torch: {torch.__version__}",
                debug=args.debug)

    # =========================
    # Handle paths
    # =========================
    # Desired RPPO model to retrained
    results_ID=""
    
    # Paths
    config_path                 = ROOT / "EBASTv2_train" / "EBASTv2_train_2.yaml"
    encounter_settings_path     = ROOT / "EBASTv2_train" / "encounter_settings.json"
    spawn_requests_bank_path    = ROOT / "EBASTv2_train" / "spawn_request_bank_1000.pkl"
    
    (model_path, train_args_log_path, 
     _, tb_path, _, checkpoint_dir)     = get_re_trained_RL_model_path(root=ROOT,results_ID=results_ID)

    # =========================
    # Instantiate the environment wrapper - parallelized
    # =========================
    # Load the spawn requests bank
    spawn_requests_bank = load_spawn_requests_bank_path(spawn_requests_bank_path)
    print_debug("[MAIN] Spawn request bank loaded",
                debug=args.debug)
    
    # DummyVecEnv and SubprocVecEnv except function, not instance, hence:
    def make_env(rank):
        def _init():
            env = EBASTv2Env(
                ROOT=ROOT,
                config_path=config_path,
                encounter_settings_path=encounter_settings_path,
                spawn_requests_bank=spawn_requests_bank,
                use_fmpy=args.use_fmpy,
                debug=args.debug
            )
            # env.reset(seed=None if args.seed is None else args.seed + rank)
            return Monitor(env)
        return _init
    
    # Vectorized the Env
    n_envs = args.n_envs
    vec_env = SubprocVecEnv([
        make_env(rank) for rank in range(n_envs)
    ])
    print_debug("[MAIN] VecEnv created",
                debug=args.debug)

    # =========================
    # Load the existing RPPO model
    # =========================
    tb_dir = tb_path if args.tensorboard_log else None
    
    old_model_path      = ROOT / "EBASTv2_train" / "trained_model" / results_ID / "model" / "model.zip"
    
    if not old_model_path.exists():
        raise FileNotFoundError(f"Existing RPPO model not found: {old_model_path}")

    recurrent_ppo_model = RecurrentPPO.load(
        str(old_model_path),
        env=vec_env,
        device=args.device,
        force_reset=True
    )
    # The loaded model may retain the TensorBoard path from its original run.
    # Explicitly redirect logging to the new continued-run folder.
    recurrent_ppo_model.tensorboard_log = tb_dir

    print(
        f"[MAIN] Existing RPPO model loaded\n"
        f"[MAIN] Source model       : {old_model_path}\n"
        f"[MAIN] Existing timesteps : {recurrent_ppo_model.num_timesteps:,}\n"
        f"[MAIN] Number of envs     : {recurrent_ppo_model.n_envs}\n"
        f"[MAIN] RPPO n_steps       : {recurrent_ppo_model.n_steps}",
        flush=True,
    )
    
    # Log training settings
    log_training_args(ROOT=ROOT, args=args, log_path=train_args_log_path, 
                      save_reward_function=args.save_reward_function)

    # =========================
    # Train the RL model. Then save the trained model
    # =========================
    learn_kwargs = {}
    
    # For checkpoint training, Record for every half of the total timesteps
    checkpoint_freq = max((args.total_timesteps // 2) // args.n_envs, 1)
    checkpoint_callback = CheckpointCallback(
        save_freq=checkpoint_freq,
        save_path=str(checkpoint_dir),
        name_prefix=args.model_name,
        save_replay_buffer=False,
        save_vecnormalize=True,
        verbose=2,
    )
    learn_kwargs["callback"] = checkpoint_callback
    
    # For tensorboard logging
    if tb_dir is not None:
        learn_kwargs["tb_log_name"] = args.model_name

    print("[MAIN] Starting learn()", flush=True)
    
    # Continue to retrain the same mode by setting reset_num_timesteps to False
    recurrent_ppo_model.learn(total_timesteps=args.total_timesteps,
                              reset_num_timesteps=False,
                              **learn_kwargs)
    
    print("[MAIN] Finished learn()", flush=True)

    recurrent_ppo_model.save(model_path)
    print(f"[MAIN] Model saved to: {model_path}", flush=True)
    
    # Close the vec_env
    vec_env.close()
    

if __name__ == "__main__":
    freeze_support()
    main()
