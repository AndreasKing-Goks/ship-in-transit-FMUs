"""
Script to enable re-training model
Support Vectorized Environment for parallel computation
"""

from pathlib import Path
import sys
import os
import argparse

# Workaround for OpenMP duplicate runtime
os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

# Important to keep to prevent sb3-contrib importing torch from ARS that causes error
import torch
print("Torch:", torch.__version__)

from sb3_contrib import RecurrentPPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.vec_env import SubprocVecEnv

# Ensure libcosim DLL is found on WINDOWS
# Note: os.add_dll_directory only exists on Windows.
# On Linux/HPC, shared libraries should be handled by LD_LIBRARY_PATH.
if sys.platform.startswith("win"):
    dll_dir = Path(sys.prefix) / "Lib" / "site-packages" / "libcosimpy" / "libcosimc"

    if dll_dir.exists():
        os.add_dll_directory(str(dll_dir))
    else:
        print(f"Warning: libcosim DLL directory not found: {dll_dir}")

## PATH HELPER (OBLIGATORY)
# project root = two levels up from this file
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from EBASTv2_core.env import EBASTv2Env
from EBASTv2_core.episode_logger import log_episode_recap, log_training_args
from EBASTv2_core.path_utils import get_RL_model_path, get_re_trained_RL_model_path
from orchestrator.scenario_config import generate_spawn_request_bank, load_spawn_requests_bank_path

import numpy as np


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


def parse_cli_args():
    parser = argparse.ArgumentParser(description="EB-ASTv2 with RecurrentPPO model")

    # Paths / run setup
    parser.add_argument("--config_path", type=Path, default=ROOT / "EBASTv2_train" / "EBASTv2_train_2.yaml", metavar="CONFIG_PATH",
                        help="PATH: training config yaml path (default: ROOT/EBASTv2_train/EBASTv2_train.yaml)")
    parser.add_argument("--encounter_settings_path", type=Path, default=ROOT / "EBASTv2_train" / "encounter_settings.json", metavar="ENCOUNTER_SETTINGS_PATH",
                        help="PATH: encounter settings json path (default: ROOT/EBASTv2_train/encounter_settings.json)")
    parser.add_argument("--spawn_requests_bank_path", type=Path, default=ROOT / "EBASTv2_train" / "spawn_request_bank.pkl", metavar="SPAWN_REQUESTS_BANK_PATH",
                        help="PATH: spawn requests bank pickle path (default: ROOT/EBASTv2_train/spawn_request_bank.pkl)")
    parser.add_argument("--model_name", type=str, default="EB-ASTv2_train_2ts", metavar="MODEL_NAME",
                        help="RUN: model/run name used for output folders (default: EB-ASTv2_train_2ts)")
    parser.add_argument("--save_anim_filename", type=str, default="EBASTv2_train.gif", metavar="SAVE_ANIM_FILENAME",
                        help="RUN: animation filename to save (default: EBASTv2_train.gif)")
    parser.add_argument("--save_reward_function", type=str2bool, default=True, metavar="SAVE_REWARD_FUNCTION",
                        help="RUN: save the reward function and automatically store it the log (default: False)")
    parser.add_argument("--results_ID", type=str, default="EB-ASTv2_train_2ts_2026-06-13_22-30-22_69f2", metavar="RESULTS_ID",
                        help="RUN: continue training the already trained model with the defined results_ID (default: EB-ASTv2_train_2ts_2026-06-13_22-30-22_69f2)")

    # Spawn request generation
    parser.add_argument("--n_cases", type=int, default=100, metavar="N_CASES",
                        help="ENV: number of spawn request cases to generate/collect (default: 100)")
    parser.add_argument("--training_case_ratio", type=float, default=0.8, metavar="TRAINING_CASE_RATIO",
                        help="ENV: ratio of cases used for training in RL env (default: 0.8)")
    parser.add_argument("--overwrite_spawn_bank", type=str2bool, default=False, metavar="OVERWRITE_SPAWN_BANK",
                        help="ENV: whether to overwrite existing spawn request bank (default: False)")
    
    # Environment vectorization
    parser.add_argument("--n_envs", type=int, default=4, metavar="N_ENVS",
                        help="VEC_ENV: The number of environment instances for computating parallelization (default: 4)")

    # RecurrentPPO core
    parser.add_argument("--total_timesteps", type=int, default=2_560_000, metavar="TOTAL_TIMESTEPS",
                        help="AST: total model training timesteps. Ideally bigger than n_steps (default: 128_000)")
    parser.add_argument("--tensorboard_log", type=str2bool, default=True, metavar="TENSORBOARD_LOG",
                        help="AST: enable tensorboard logging to training folder (default: True)")
    parser.add_argument("--seed", type=int, default=None, metavar="SEED",
                        help="AST: random seed (default: None)")
    parser.add_argument("--device", type=str, default="cuda", metavar="DEVICE",
                        help="AST: device to use, e.g. cpu, cuda, auto (default: cuda)")

    return parser.parse_args()


def main():
    args = parse_cli_args()

    # =========================
    # Handle paths
    # =========================
    (model_path, train_args_log_path, 
     episode_log_path, tb_path, 
     saved_animation_path, checkpoint_dir) = get_re_trained_RL_model_path(
        root=ROOT,
        results_ID=args.results_ID,
        model_name=args.model_name,
        save_anim_filename=args.save_anim_filename,
    )

    # =========================
    # Instantiate the environment wrapper - parallelized
    # =========================
    spawn_requests_bank_path = generate_spawn_request_bank(
        ROOT=ROOT,
        config_path=args.config_path,
        encounter_settings_path=args.encounter_settings_path,
        spawn_requests_bank_path=args.spawn_requests_bank_path,
        n_cases=args.n_cases,
        training_case_ratio=args.training_case_ratio,
        overwrite=args.overwrite_spawn_bank,
    )
    spawn_requests_bank = load_spawn_requests_bank_path(spawn_requests_bank_path)
    
    # DummyVecEnv and SubprocVecEnv except function, not instance, hence:
    def make_env(rank):
        def _init():
            env = EBASTv2Env(
                ROOT=ROOT,
                config_path=args.config_path,
                encounter_settings_path=args.encounter_settings_path,
                spawn_requests_bank=spawn_requests_bank,
            )
            env.reset(seed=None if args.seed is None else args.seed + rank)
            
            return Monitor(env)
        return _init
    
    # Vectorized the Env
    n_envs = args.n_envs
    vec_env = SubprocVecEnv([
        make_env(rank) for rank in range(n_envs)
    ])

    # =========================
    # Instantiate the RL Model
    # =========================
    tb_dir = tb_path if args.tensorboard_log else None
    
    old_model_path = ROOT / "EBASTv2_train" / "trained_model" / args.results_ID / "model" / "model.zip"
    
    if not old_model_path.exists():
        raise FileNotFoundError(f"Old model not found: {old_model_path}")
    
    recurrent_ppo_model = RecurrentPPO.load(
        str(old_model_path),
        env=vec_env,
        device=args.device,
        tensorboard_log=tb_dir
    )
    
    # Log training settings
    log_training_args(ROOT=ROOT, args=args, log_path=train_args_log_path, 
                      save_reward_function=args.save_reward_function)

    # =========================
    # Train the RL model. Then save the trained model
    # =========================
    learn_kwargs = {}
    
    # For checkpoint training, Record for every one-fourth of the total timesteps
    checkpoint_freq = max((args.total_timesteps // 4) // args.n_envs, 1)
    checkpoint_callback = CheckpointCallback(
        save_freq=checkpoint_freq,
        save_path=str(checkpoint_dir),
        name_prefix=args.model_name,
        save_replay_buffer=False,
        save_vecnormalize=True
    )
    learn_kwargs["callback"] = checkpoint_callback
    
    # For tensorboard logging
    if tb_dir is not None:
        learn_kwargs["tb_log_name"] = args.model_name

    recurrent_ppo_model.learn(total_timesteps=args.total_timesteps, 
                              reset_num_timesteps=False,
                              **learn_kwargs)

    recurrent_ppo_model.save(model_path)
    vec_env.close()

if __name__ == "__main__":
    main()
