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
import torch

from stable_baselines3 import SAC
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
from EBASTv2_core.episode_logger import log_episode_recap, log_training_args
from EBASTv2_core.path_utils import get_RL_model_path
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
    parser = argparse.ArgumentParser(description="EB-ASTv2 with Soft Actor-Critic model")

    # Run setup
    parser.add_argument("--model_name", type=str, default="EB-ASTv2_train_sac", metavar="MODEL_NAME",
                        help="RUN: model/run name used for output folders (default: EB-ASTv2_train_sac)")
    parser.add_argument("--save_anim_filename", type=str, default="EBASTv2_train_sac.gif", metavar="SAVE_ANIM_FILENAME",
                        help="RUN: animation filename to save (default: EBASTv2_train_sac.gif)")
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

    # Soft Actor-Critic core
    parser.add_argument('--total_timesteps', type=int, default=10_000_000, metavar='TOTAL_TIMESTEPS',
                        help='AST: total timesteps for overall AST training [start_steps + train_steps] (default=10_000_000)')
    parser.add_argument("--policy", type=str, default="MultiInputPolicy", metavar="POLICY",
                        help="AST: SAC policy name (default: MultiInputPolicy)")
    parser.add_argument('--learning_rate', type=float, default=3e-4, metavar='LEARNING_RATE',
                        help='AST: learning rate for adam optimizer (default: 3e-4)')
    parser.add_argument('--buffer_size', type=int, default=1_280_000, metavar='REPLAY_BUFFER_SIZE',
                        help='AST: size of the replay buffer (default: 1_280_000)')
    parser.add_argument('--learning_starts', type=int, default=128_000, metavar='LEARNING_STARTS',
                        help='AST: how many steps of the model to collect transitions for before learning starts (default: 128_000)')
    parser.add_argument('--batch_size', type=int, default=1024, metavar='BATCH_SIZE',
                        help='AST: minibatch size for each gradient update (default: 1024)')
    parser.add_argument('--tau', type=float, default=0.005, metavar='SOFT_UPDATE_COEFFICIENT',
                        help='AST: the soft update coefficient [“Polyak update”, between 0 and 1] (default: 0.005)')
    parser.add_argument('--gamma', type=float, default=1.00, metavar='DISCOUNT_FACTOR',
                        help='AST: RL discount factor (default: 1.00)')
    parser.add_argument('--train_freq', type=int, default=10, metavar='TRAIN_FREQ',
                        help='AST: Update the model every train_freq steps. \
                            alternatively pass a tuple of frequency and unit like (5, "step") or (2, "episode") (default: 10)')
    parser.add_argument('--gradient_steps', type=int, default=64, metavar='GRADIENT_STEPS',
                        help='AST: How many gradient steps to do after each rollout (see train_freq). \
                            Set to -1 means to do as many gradient steps as steps done in the environment during the rollout (default: 64)')
    parser.add_argument('--n_steps', type=int, default=1, metavar='N_STEPS',
                        help='AST: When n_steps > 1, uses n-step return (with the NStepReplayBuffer) when updating the Q-value network (default:1)')
    parser.add_argument('--ent_coef', type=str, default="auto", metavar='ENT_COEF',
                        help='AST: Entropy regularization coefficient. (Equivalent to inverse of reward scale in the original SAC paper.)\
                            Controlling exploration/exploitation trade-off. Set it to "auto" to learn it automatically \
                            (and "auto_0.1" for using 0.1 as initial value) (default: "auto")')
    parser.add_argument('--target_update_interval', type=int, default=1, metavar='TARGET_UPDATE_INTERVAL',
                        help='AST: update the target network every target_network_update_freq gradient steps (default: 1)')
    parser.add_argument('--target_entropy', type=str, default="auto", metavar='TARGET_ENTROPY',
                        help='AST: target entropy when learning ent_coef. Can be set to auto (default: "auto")')
    parser.add_argument('--stats_window_size', type=int, default=25, metavar='TARGET_UPDATE_INTERVAL',
                        help='AST: window size for the rollout logging, specifying the number of episodes to average \
                            the reported success rate, mean episode length, and mean reward over (default: 25)')
    parser.add_argument('--tensorboard_log', type=str2bool, default=True, metavar='TENSORBOARD_LOG',
                        help='AST: do tensorboard log. The log will be stored inside the training folder (default: True)')
    parser.add_argument('--verbose', type=int, default=1, metavar='VERBOSE',
                        help='AST: verbosity level: 0 for no output, 1 for info messages (such as device or wrappers used), \
                            2 for debug messages (default: 1)')
    parser.add_argument('--seed', type=int, default=None, metavar='SEED',
                        help='AST: seed for the pseudo random generators (default: None)')
    parser.add_argument('--device', type=str, default="cuda", metavar='DEVICE',
                        help='AST: device (cpu, cuda, …) on which the code should be run. \
                            Setting it to auto, the code will be run on the GPU if possible. (default: "cuda)')

    # Post-training run / plotting
    parser.add_argument("--run_trained_episode", type=str2bool, default=False, metavar="RUN_TRAINED_EPISODE",
                        help="RESULT: run the trained model once after saving/loading (default: False)")
    parser.add_argument("--animate", type=str2bool, default=False, metavar="ANIMATE",
                        help="RESULT: create/show/save trajectory animation (default: False)")
    parser.add_argument("--plot", type=str2bool, default=False, metavar="PLOT",
                        help="RESULT: plot fleet trajectory after animation (default: False)")
    parser.add_argument("--animation_show", type=str2bool, default=False, metavar="ANIMATION_SHOW",
                        help="RESULT: show animation window (default: False)")
    parser.add_argument("--animation_block", type=str2bool, default=False, metavar="ANIMATION_BLOCK",
                        help="RESULT: block execution while showing animation (default: False)")

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
    # Paths
    config_path                 = ROOT / "EBASTv2_train" / "EBASTv2_train_2.yaml"
    encounter_settings_path     = ROOT / "EBASTv2_train" / "encounter_settings.json"
    spawn_requests_bank_path    = ROOT / "EBASTv2_train" / "spawn_request_bank_1000.pkl"
    
    (model_path, train_args_log_path, 
     episode_log_path, tb_path, 
     saved_animation_path, checkpoint_dir) = get_RL_model_path(
        root=ROOT,
        model_name=args.model_name,
        save_anim_filename=args.save_anim_filename,
    )

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
    # Instantiate the RL Model
    # =========================
    tb_dir = tb_path if args.tensorboard_log else None

    sac_model = SAC(
        policy=args.policy,
        env=vec_env,
        learning_rate=args.learning_rate,
        buffer_size=args.buffer_size,
        learning_starts=args.learning_starts,
        batch_size=args.batch_size,
        tau=args.tau,
        gamma=args.gamma,
        train_freq=(args.train_freq, "step"),
        gradient_steps=args.gradient_steps,
        n_steps=args.n_steps,
        ent_coef=args.ent_coef,
        target_update_interval=args.target_update_interval,
        target_entropy=args.target_entropy,
        stats_window_size=args.stats_window_size,
        tensorboard_log=tb_dir,
        verbose=args.verbose,
        seed=args.seed,
        device=args.device
    )
    print_debug("[MAIN] SAC model created",
                debug=args.debug)
    
    # Log training settings
    log_training_args(ROOT=ROOT, args=args, log_path=train_args_log_path, 
                      save_reward_function=args.save_reward_function)

    # =========================
    # Train the RL model. Then save the trained model
    # =========================
    learn_kwargs = {}
    
    # For checkpoint training, Record for every one-fifth of the total timesteps
    checkpoint_freq = max((args.total_timesteps // 5) // args.n_envs, 1)
    checkpoint_callback = CheckpointCallback(
        save_freq=checkpoint_freq,
        save_path=str(checkpoint_dir),
        name_prefix=args.model_name,
        save_replay_buffer=False,
        save_vecnormalize=True,
        verbose=2
    )
    learn_kwargs["callback"] = checkpoint_callback
    
    # For tensorboard logging
    if tb_dir is not None:
        learn_kwargs["tb_log_name"] = args.model_name

    # =========================
    # Chunk Training
    # =========================
    
    target_timesteps    = args.total_timesteps
    chunk_timesteps     = args.chunk_timesteps
    chunk_idx           = 0
    
    print(
        f"[MAIN] Starting chunked training\n"
        f"[MAIN] Target timesteps : {target_timesteps:,}\n"
        f"[MAIN] Chunk timesteps  : {chunk_timesteps:,}\n"
        f"[MAIN] Number of envs   : {args.n_envs}",
        flush=True)
    
    try:
        while sac_model.num_timesteps < target_timesteps:
            # Advance the chunk training index
            chunk_idx += 1
            
            # Compute the remaining timesteps before the target timesteps
            remaining_timesteps = (target_timesteps - sac_model.num_timesteps)
            
            # Do not intentionally requests more than what remains
            current_chunk_timesteps = min(chunk_timesteps, remaining_timesteps)
            
            print(
                f"\n[MAIN] ===============================\n"
                f"[MAIN] Starting chunk {chunk_idx}\n"
                f"[MAIN] Current timesteps   : "
                f"{sac_model.num_timesteps:,}\n"
                f"[MAIN] Requested this chunk: "
                f"{current_chunk_timesteps:,}\n"
                f"[MAIN] Remaining to target : "
                f"{remaining_timesteps:,}\n"
                f"[MAIN] ===============================",
                flush=True,
            )
            
            # Continue the training the SAME model
            sac_model.learn(total_timesteps=current_chunk_timesteps,
                            reset_num_timesteps=False,
                            **learn_kwargs)
            
            print(
                f"[MAIN] Chunk {chunk_idx} finished at ",
                f"{sac_model.num_timesteps:,} timesteps", 
                flush=True
            )
            
            # Stop if target has been reached
            if sac_model.num_timesteps >= target_timesteps:
                break
            
            # -------------------------
            # Recycle all environment worker processes
            # -------------------------
            print(
                "[MAIN] Closing current VecEnv workers...",
                flush=True,
            )

            vec_env.close()

            print(
                "[MAIN] Creating fresh VecEnv workers...",
                flush=True,
            )

            vec_env = make_vec_env()

            # Attach the fresh worker pool to the SAME model
            sac_model.set_env(
                vec_env,
                force_reset=True,
            )

            print(
                "[MAIN] Fresh VecEnv attached to model",
                flush=True,
            )
            
    finally:        
        # Always clean up the final environment pool
        print("[MAIN] Closing final VecEnv...", flush=True)
        
        vec_env.close()
        
    # Final save after the end of the training
    sac_model.save(model_path)
    print(f"[MAIN] Model saved to: {model_path}", flush=True)
        
    print(
        f"[MAIN] Training finished at "
        f"{sac_model.num_timesteps:,} timesteps",
        flush=True,
    )

    # =========================
    # Run the trained model and log the episode
    # =========================
    if args.run_trained_episode:
        del sac_model
        sac_model = SAC.load(model_path)
        
        eval_env = EBASTv2Env(
            ROOT=ROOT,
            config_path=args.config_path,
            encounter_settings_path=args.encounter_settings_path,
            spawn_requests_bank=spawn_requests_bank,
            use_fmpy=args.use_fmpy,
            debug=args.debug
        )

        obs, _ = eval_env.reset()

        while True:
            action, _ = sac_model.predict(
                obs,
                deterministic=True,
            )
            obs, _, terminated, truncated, _ = eval_env.step(action)

            if terminated or truncated:
                break
        
        # Log the simulation episode using the trained policy
        log_episode_recap(env=eval_env, log_path=episode_log_path)

    # =========================
    # Animation and Plot
    # =========================
    if args.animate and args.run_trained_episode:
            eval_env.instance.AnimateFleetTrajectory(
                ship_ids=None,
                show=args.animation_show,
                block=args.animation_block,
                mode="quick",
                fig_width=10.0,
                margin_frac=0.08,
                equal_aspect=True,
                interval_ms=20,
                frame_step=10,
                trail_len=50,
                plot_routes=True,
                plot_waypoints=True,
                plot_roa=True,
                plot_start_end=True,
                plot_inter_wp_roa=True,
                plot_inter_wp_proj=False,
                with_labels=True,
                precompute_ship_outlines=True,
                save_path=saved_animation_path,
                writer_fps=20,
                palette=None,
                blit=True,
                ship_scale=1.0,
            )

    if args.plot and args.run_trained_episode:
            eval_env.instance.PlotFleetTrajectory(mode="quick", ship_scale=1.0)


if __name__ == "__main__":
    freeze_support()
    main()
