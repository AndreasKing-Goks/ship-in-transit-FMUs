from pathlib import Path
import sys
import os

# Workaround for OpenMP duplicate runtime
os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

# Important to keep to preven sb3-contrib importing torch from ARS that causes error
import torch
print("Torch:", torch.__version__)

from sb3_contrib import RecurrentPPO
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.monitor import Monitor
from gymnasium.utils.env_checker import check_env

# Ensure libcosim DLL is found
dll_dir = Path(sys.prefix) / "Lib" / "site-packages" / "libcosimpy" / "libcosimc"
os.add_dll_directory(str(dll_dir))

## PATH HELPER (OBLIGATORY)
# project root = two levels up from this file
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from EBASTv2_core.env import EBASTv2Env
from EBASTv2_core.episode_logger import log_episode_recap
from EBASTv2_core.path_utils import get_RL_model_path
from orchestrator.scenario_config import generate_spawn_request_bank, load_spawn_requests_bank_path

import numpy as np
import time

# =========================
# Handle paths
# =========================
# Get the config path
config_path                     = ROOT / "EBASTv2_train" / "EBASTv2_train.yaml"

# Get the encounter settings path
encounter_settings_path         = ROOT / "EBASTv2_train" / "encounter_settings.json"

# Spawn requests bank path
spawn_requests_bank_path        = ROOT / "EBASTv2_train" / "spawn_request_bank.pkl"

# Get the RL-process related paths
model_name                      = "EBASTv2_train"
(model_path, _, 
 episode_log_path, tb_path, 
 saved_animation_path, _)       = get_RL_model_path(root=ROOT, model_name=model_name, save_anim_filename="EBASTv2_train.gif")

# =========================
# Instantiate the environment wrapper
# =========================
# Generates/collect spawn requests
spawn_requests_bank_path        = generate_spawn_request_bank(ROOT=ROOT,
                                                              config_path=config_path,
                                                              encounter_settings_path=encounter_settings_path,
                                                              spawn_requests_bank_path=spawn_requests_bank_path,
                                                              n_cases=100,
                                                              training_case_ratio=0.8,                              # Specifically for RL-env
                                                              overwrite=False)
spawn_requests_bank             = load_spawn_requests_bank_path(spawn_requests_bank_path)

# Instantiate the RL-environment wrapper class
env = EBASTv2Env(
    ROOT=ROOT,
    config_path=config_path,
    encounter_settings_path=encounter_settings_path,
    spawn_requests_bank=spawn_requests_bank
    )

# Check the env validity
try_check_env = True
if try_check_env:
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
        sys.exit(1)  # non-zero exit code stops the script


# =========================
# Instantiate the RL Model
# =========================
# Use MultiInputLstmPolicy instead of MlpLstmPolicy to enable working with
# dictionary-based observation space
recurrent_ppo_model = RecurrentPPO(policy="MultiInputLstmPolicy",
                                   env=Monitor(env),
                                   learning_rate=3e-4,
                                   n_steps=128,
                                   batch_size=128,
                                   n_epochs=10,
                                   gamma=1.00,                          # For AST purposes
                                   gae_lambda=0.95,
                                   clip_range=0.2,
                                   clip_range_vf=None,
                                   normalize_advantage=True,
                                   ent_coef=0.0,
                                   vf_coef=0.5,
                                   max_grad_norm=0.5,
                                   use_sde=False,
                                   sde_sample_freq=-1,
                                   target_kl=None,
                                   stats_window_size=100,
                                   tensorboard_log=tb_path,
                                   policy_kwargs=None,
                                   verbose=0,
                                   seed=None,
                                   device="cuda")

# =========================
# Train the RL model. Then save the trained model
# =========================
# Train while counting the timer
start_time      = time.time()
recurrent_ppo_model.learn(total_timesteps=5)
elapsed_time    = time.time() - start_time

# Convert the elapsed time to hours, minutes, seconds
raw_minutes, seconds    = divmod(elapsed_time, 60)
hours, minutes          = divmod(raw_minutes, 60)
train_time              = (hours, minutes, seconds)

# Save the model
recurrent_ppo_model.save(model_path)

# =========================
# Run the trained model and log the episode
# =========================
# Remove the model
del recurrent_ppo_model

# Load the trained model
recurrent_ppo_model = RecurrentPPO.load(model_path)

# Reset the trained model
obs, info   = env.reset()

# Cell and hidden state of the LSTM
lstm_states = None
num_envs    = 1

# Episode start signals are used to reset the lstm states
episode_starts = np.ones((num_envs,), dtype=bool)
while True:
    action, lstm_states = recurrent_ppo_model.predict(obs,
                                                      state=lstm_states,
                                                      episode_start=episode_starts,
                                                      deterministic=True)
    obs, rewards, terminated, truncated, info = env.step(action)
    episode_starts = terminated or truncated
    
    # Break the loop if it's either terminated or truncated
    if episode_starts:
        break
        
# Log the simulation episode using the trained policy
log_episode_recap(env=env, log_path=episode_log_path)

# =========================
# Animation and Plot
# =========================
# Available formats:
# - .mp4
# - .gif
# - .avi
# - .mov

# Animate Simulation
env.instance.AnimateFleetTrajectory(
        ship_ids=None,
        show=True,
        block=True,
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
        ship_scale=1.0
    )

# Plot Trajectory
env.instance.PlotFleetTrajectory(mode="quick", ship_scale=1.0)