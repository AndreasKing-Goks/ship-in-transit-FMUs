from pathlib import Path
import sys
import os

# Workaround for OpenMP duplicate runtime
os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

# Important to keep to preven sb3-contrib importing torch from ARS that causes error
import torch
print("Torch:", torch.__version__)

from sb3_contrib import RecurrentPPO

# Ensure libcosim DLL is found
dll_dir = Path(sys.prefix) / "Lib" / "site-packages" / "libcosimpy" / "libcosimc"
os.add_dll_directory(str(dll_dir))

## PATH HELPER (OBLIGATORY)
# project root = two levels up from this file
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from EBASTv2_core.env import EBASTv2Env
from EBASTv2_core.episode_logger import log_episode_recap
from orchestrator.scenario_config import generate_spawn_request_bank, load_spawn_requests_bank_path

import numpy as np

# =========================
# Handle paths
# =========================
# Trained Model Name
model_name                      = "EB-ASTv2_train_2ts_continue_03_2026-06-13_22-30-22_69f2"

# Get the config path
config_path                     = ROOT / "EBASTv2_train" / "EBASTv2_train_2.yaml"

# Get the encounter settings path
encounter_settings_path         = ROOT / "EBASTv2_train" / "encounter_settings.json"

# Spawn requests bank path
spawn_requests_bank_path        = ROOT / "EBASTv2_train" / "spawn_request_bank.pkl"

# Get the trianed model
model_path                      = ROOT / "EBASTv2_train" / "trained_model" / model_name / "model" / "model.zip"

# Log path
log_path                        = ROOT / "EBASTv2_train" / "simulated_trained_model" / "episode_recap.txt"
# log_path                        = ROOT / "EBASTv2_train" / "trained_model" / model_name / "log" / "episode_recap.txt"

# Get the save path for animation
saved_animation_path            = ROOT / "EBASTv2_train" / "simulated_trained_model" / "simulated_trained_model.gif"
# saved_animation_path            = ROOT / "EBASTv2_train" / "trained_model" / model_name / "saved_animation" / "simulated_trained_model.mp4"

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

# =========================
# Run the trained model and log the episode
# =========================
# Set the environment to evaluation mode
env.set_for_evaluation()

# Load the trained model
recurrent_ppo_model = RecurrentPPO.load(model_path)

# Reset the trained model
case_idx    = None
obs, info   = env.reset(specific_case_idx=case_idx)

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
        
# Log the episodes
log_episode_recap(env=env, log_path=log_path)
print(f"Episode recap saved to: {log_path}")

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
        plot_inter_wp_roa=False,
        plot_inter_wp_proj=False,
        with_labels=True,
        precompute_ship_outlines=True,
        # save_path=saved_animation_path,
        writer_fps=20,
        palette=None,
        blit=True,
        ship_scale=1.0
    )

# Plot Trajectory
env.instance.PlotFleetTrajectory(mode="quick", ship_scale=1.0)