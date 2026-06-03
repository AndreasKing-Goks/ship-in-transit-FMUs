from pathlib import Path
import sys
import os
import argparse
import time

# Workaround for OpenMP duplicate runtime
os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

# Important to keep to prevent sb3-contrib importing torch from ARS that causes error
import torch
print("Torch:", torch.__version__)

from sb3_contrib import RecurrentPPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import CheckpointCallback
from gymnasium.utils.env_checker import check_env

# Ensure libcosim DLL is found
# Note: os.add_dll_directory only exists on Windows.
dll_dir = Path(sys.prefix) / "Lib" / "site-packages" / "libcosimpy" / "libcosimc"
if hasattr(os, "add_dll_directory") and dll_dir.exists():
    os.add_dll_directory(str(dll_dir))

## PATH HELPER (OBLIGATORY)
# project root = two levels up from this file
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from EBASTv2_core.env import EBASTv2Env
from EBASTv2_core.episode_logger import log_episode_recap, log_training_args
from EBASTv2_core.path_utils import get_RL_model_path
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
    parser.add_argument("--config_path", type=Path, default=ROOT / "EBASTv2_train" / "EBASTv2_train.yaml", metavar="CONFIG_PATH",
                        help="PATH: training config yaml path (default: ROOT/EBASTv2_train/EBASTv2_train.yaml)")
    parser.add_argument("--encounter_settings_path", type=Path, default=ROOT / "EBASTv2_train" / "encounter_settings.json", metavar="ENCOUNTER_SETTINGS_PATH",
                        help="PATH: encounter settings json path (default: ROOT/EBASTv2_train/encounter_settings.json)")
    parser.add_argument("--spawn_requests_bank_path", type=Path, default=ROOT / "EBASTv2_train" / "spawn_request_bank.pkl", metavar="SPAWN_REQUESTS_BANK_PATH",
                        help="PATH: spawn requests bank pickle path (default: ROOT/EBASTv2_train/spawn_request_bank.pkl)")
    parser.add_argument("--model_name", type=str, default="EB-ASTv2_train", metavar="MODEL_NAME",
                        help="RUN: model/run name used for output folders (default: EB-ASTv2_train)")
    parser.add_argument("--save_anim_filename", type=str, default="EBASTv2_train.gif", metavar="SAVE_ANIM_FILENAME",
                        help="RUN: animation filename to save (default: EBASTv2_train.gif)")
    parser.add_argument("--save_reward_function", type=str2bool, default=True, metavar="SAVE_REWARD_FUNCTION",
                        help="RUN: save the reward function and automatically store it the log (default: False)")

    # Spawn request generation
    parser.add_argument("--n_cases", type=int, default=100, metavar="N_CASES",
                        help="ENV: number of spawn request cases to generate/collect (default: 100)")
    parser.add_argument("--training_case_ratio", type=float, default=0.8, metavar="TRAINING_CASE_RATIO",
                        help="ENV: ratio of cases used for training in RL env (default: 0.8)")
    parser.add_argument("--overwrite_spawn_bank", type=str2bool, default=False, metavar="OVERWRITE_SPAWN_BANK",
                        help="ENV: whether to overwrite existing spawn request bank (default: False)")
    parser.add_argument("--try_check_env", type=str2bool, default=False, metavar="TRY_CHECK_ENV",
                        help="ENV: run gymnasium check_env before training (default: True)")

    # RecurrentPPO core
    parser.add_argument("--total_timesteps", type=int, default=128_000, metavar="TOTAL_TIMESTEPS",
                        help="AST: total model training timesteps. Ideally bigger than n_steps (default: 128_000)")
    parser.add_argument("--policy", type=str, default="MultiInputLstmPolicy", metavar="POLICY",
                        help="AST: RecurrentPPO policy name (default: MultiInputLstmPolicy)")
    parser.add_argument("--learning_rate", type=float, default=3e-4, metavar="LEARNING_RATE",
                        help="AST: learning rate (default: 3e-4)")
    parser.add_argument("--n_steps", type=int, default=128, metavar="N_STEPS",
                        help="AST: number of steps to run for each environment per update (default: 128)")
    parser.add_argument("--batch_size", type=int, default=128, metavar="BATCH_SIZE",
                        help="AST: minibatch size (default: 128)")
    parser.add_argument("--n_epochs", type=int, default=10, metavar="N_EPOCHS",
                        help="AST: number of epoch when optimizing surrogate loss (default: 10)")
    parser.add_argument("--gamma", type=float, default=1.00, metavar="GAMMA",
                        help="AST: discount factor (default: 1.00)")
    parser.add_argument("--gae_lambda", type=float, default=0.95, metavar="GAE_LAMBDA",
                        help="AST: GAE lambda parameter (default: 0.95)")
    parser.add_argument("--clip_range", type=float, default=0.2, metavar="CLIP_RANGE",
                        help="AST: PPO clipping parameter (default: 0.2)")
    parser.add_argument("--normalize_advantage", type=str2bool, default=True, metavar="NORMALIZE_ADVANTAGE",
                        help="AST: normalize advantage estimates (default: True)")
    parser.add_argument("--ent_coef", type=float, default=0.0, metavar="ENT_COEF",
                        help="AST: entropy coefficient (default: 0.0)")
    parser.add_argument("--vf_coef", type=float, default=0.5, metavar="VF_COEF",
                        help="AST: value function coefficient (default: 0.5)")
    parser.add_argument("--max_grad_norm", type=float, default=0.5, metavar="MAX_GRAD_NORM",
                        help="AST: maximum gradient norm (default: 0.5)")
    parser.add_argument("--use_sde", type=str2bool, default=False, metavar="USE_SDE",
                        help="AST: use generalized State Dependent Exploration (default: False)")
    parser.add_argument("--sde_sample_freq", type=int, default=-1, metavar="SDE_SAMPLE_FREQ",
                        help="AST: SDE sample frequency (default: -1)")
    parser.add_argument("--stats_window_size", type=int, default=100, metavar="STATS_WINDOW_SIZE",
                        help="AST: rollout logging window size (default: 100)")
    parser.add_argument("--tensorboard_log", type=str2bool, default=True, metavar="TENSORBOARD_LOG",
                        help="AST: enable tensorboard logging to training folder (default: True)")
    parser.add_argument("--verbose", type=int, default=0, metavar="VERBOSE",
                        help="AST: verbosity level (default: 0)")
    parser.add_argument("--seed", type=int, default=None, metavar="SEED",
                        help="AST: random seed (default: None)")
    parser.add_argument("--device", type=str, default="cuda", metavar="DEVICE",
                        help="AST: device to use, e.g. cpu, cuda, auto (default: cuda)")

    # Post-training run / plotting
    parser.add_argument("--run_trained_episode", type=str2bool, default=True, metavar="RUN_TRAINED_EPISODE",
                        help="RESULT: run the trained model once after saving/loading (default: True)")
    parser.add_argument("--animate", type=str2bool, default=True, metavar="ANIMATE",
                        help="RESULT: create/show/save trajectory animation (default: True)")
    parser.add_argument("--plot", type=str2bool, default=True, metavar="PLOT",
                        help="RESULT: plot fleet trajectory after animation (default: True)")
    parser.add_argument("--animation_show", type=str2bool, default=True, metavar="ANIMATION_SHOW",
                        help="RESULT: show animation window (default: True)")
    parser.add_argument("--animation_block", type=str2bool, default=True, metavar="ANIMATION_BLOCK",
                        help="RESULT: block execution while showing animation (default: True)")

    return parser.parse_args()


def main():
    args = parse_cli_args()

    # =========================
    # Handle paths
    # =========================
    (model_path, train_args_log_path, 
     episode_log_path, tb_path, 
     saved_animation_path, checkpoint_dir) = get_RL_model_path(
        root=ROOT,
        model_name=args.model_name,
        save_anim_filename=args.save_anim_filename,
    )

    # =========================
    # Instantiate the environment wrapper
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

    env = EBASTv2Env(
        ROOT=ROOT,
        config_path=args.config_path,
        encounter_settings_path=args.encounter_settings_path,
        spawn_requests_bank=spawn_requests_bank,
    )

    if args.try_check_env:
        try:
            check_env(env)
            print("Environment passes all chekcs!")
        except Exception as e:
            print(f"Environment has issues: {e}")

            print("Start debugging observation")
            obs, info = env.reset()

            for key, space in env.observation_space.spaces.items():
                value = obs[key]
                ok = space.contains(value)

                print("\n", key)
                print("contains:", ok)
                print("shape:", value.shape, "expected:", space.shape)
                print("dtype:", value.dtype, "expected:", space.dtype)
                print("min/max:", np.min(value), np.max(value))
                print("value:", value)

            print("ABORT TRAINING")
            sys.exit(1)

    # =========================
    # Instantiate the RL Model
    # =========================
    tb_dir = tb_path if args.tensorboard_log else None

    recurrent_ppo_model = RecurrentPPO(
        policy=args.policy,
        env=Monitor(env),
        learning_rate=args.learning_rate,
        n_steps=args.n_steps,
        batch_size=args.batch_size,
        n_epochs=args.n_epochs,
        gamma=args.gamma,
        gae_lambda=args.gae_lambda,
        clip_range=args.clip_range,
        clip_range_vf=None,
        normalize_advantage=args.normalize_advantage,
        ent_coef=args.ent_coef,
        vf_coef=args.vf_coef,
        max_grad_norm=args.max_grad_norm,
        use_sde=args.use_sde,
        sde_sample_freq=args.sde_sample_freq,
        target_kl=None,
        stats_window_size=args.stats_window_size,
        tensorboard_log=tb_dir,
        policy_kwargs=None,
        verbose=args.verbose,
        seed=args.seed,
        device=args.device,
    )
    
    # Log training settings
    log_training_args(ROOT=ROOT, args=args, log_path=train_args_log_path, 
                      save_reward_function=args.save_reward_function)

    # =========================
    # Train the RL model. Then save the trained model
    # =========================
    learn_kwargs = {}
    
    # For checkpoint training, Record for every one-fourth of the total timesteps
    checkpoint_freq     = max(args.total_timesteps // 4, 1)
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

    start_time = time.time()
    recurrent_ppo_model.learn(total_timesteps=args.total_timesteps, **learn_kwargs)
    elapsed_time = time.time() - start_time

    raw_minutes, seconds = divmod(elapsed_time, 60)
    hours, minutes = divmod(raw_minutes, 60)
    train_time = (hours, minutes, seconds)

    recurrent_ppo_model.save(model_path)

    # =========================
    # Run the trained model and log the episode
    # =========================
    if args.run_trained_episode:
        del recurrent_ppo_model
        recurrent_ppo_model = RecurrentPPO.load(model_path)
        
        eval_env = EBASTv2Env(
            ROOT=ROOT,
            config_path=args.config_path,
            encounter_settings_path=args.encounter_settings_path,
            spawn_requests_bank=spawn_requests_bank,
        )

        obs, info = eval_env.reset()
        lstm_states = None
        num_envs = 1
        episode_starts = np.ones((num_envs,), dtype=bool)

        while True:
            action, lstm_states = recurrent_ppo_model.predict(
                obs,
                state=lstm_states,
                episode_start=episode_starts,
                deterministic=True,
            )
            obs, rewards, terminated, truncated, info = eval_env.step(action)
            episode_starts = terminated or truncated

            if episode_starts:
                break
        
        # Log the simulation episode using the trained policy
        log_episode_recap(env=eval_env, log_path=episode_log_path)

    print(f"Training time: {train_time[0]:.0f}h {train_time[1]:.0f}m {train_time[2]:.2f}s")
    print(f"Model saved to: {model_path}")

    # =========================
    # Animation and Plot
    # =========================
    if args.animate:
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

    if args.plot:
        eval_env.instance.PlotFleetTrajectory(mode="quick", ship_scale=1.0)


if __name__ == "__main__":
    main()
