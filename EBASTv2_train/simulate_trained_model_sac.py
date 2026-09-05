from pathlib import Path
import sys
import os

# Workaround for OpenMP duplicate runtime
os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

# Important to keep to preven sb3-contrib importing torch from ARS that causes error
import torch
print("Torch:", torch.__version__)

from stable_baselines3 import SAC

# Ensure libcosim DLL is found
dll_dir = Path(sys.prefix) / "Lib" / "site-packages" / "libcosimpy" / "libcosimc"
os.add_dll_directory(str(dll_dir))

## PATH HELPER (OBLIGATORY)
# project root = two levels up from this file
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from EBASTv2_core.env import EBASTv2Env
from EBASTv2_core.episode_logger import log_episode_recap
from EBASTv2_train.evaluate_failure_cases import evaluate_failure_cases
from orchestrator.scenario_config import load_spawn_requests_bank_path

import numpy as np

# =========================
# Handle paths
# =========================
# Trained Model Name
model_name                      = "EB-ASTv2_train_sac_2026-08-22_17-07-40_dd1b"

# Get the config path
config_path                     = ROOT / "EBASTv2_train" / "EBASTv2_train_2.yaml"

# Get the encounter settings path
encounter_settings_path         = ROOT / "EBASTv2_train" / "encounter_settings.json"

# Spawn requests bank path
spawn_requests_bank_path        = ROOT / "EBASTv2_train" / "spawn_request_bank_ebastv2.pkl"

# Get the trained model
model_path                      = ROOT / "EBASTv2_train" / "trained_model" / model_name / "model" / "model.zip"
# model_path                      = ROOT / "EBASTv2_train" / "trained_model" / "checkpoints" / "EB-ASTv2_train_sac_8000000_steps.zip"

# Log path
log_path                        = ROOT / "EBASTv2_train" / "simulated_trained_model" / "episode_recap_sac.txt"

# Evaluation recap
recap_path                      = ROOT / "EBASTv2_train" / "simulated_trained_model" / "evaluation_recap_sac_1.txt"

# =========================
# Instantiate the environment wrapper
# =========================
# Generates/collect spawn requests
spawn_requests_bank             = load_spawn_requests_bank_path(spawn_requests_bank_path)

# Instantiate the RL-environment wrapper class
env = EBASTv2Env(
    ROOT=ROOT,
    config_path=config_path,
    encounter_settings_path=encounter_settings_path,
    spawn_requests_bank=spawn_requests_bank,
    use_fmpy=True
    )

# =========================
# Load the trained model
# =========================
# Load the trained model
sac_model = SAC.load(model_path)

# =========================
# Run the trained model and log the episode
# =========================
simulate    = False
# simulate    = True

if simulate:
    # Set the environment to evaluation mode
    # env.set_for_evaluation()
    
    # Reset the trained model
    case_idx    = 89 #None
    seed        = None
    obs, _      = env.reset(seed=seed, specific_case_idx=case_idx)

    # Episode start signals are used to reset the states
    while True:
        action, _ = sac_model.predict(obs,
                                    deterministic=True)
        obs, _, terminated, truncated, _ = env.step(action)
        
        # Break the loop if it's either terminated or truncated
        if terminated or truncated:
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

    # Get the save path for animation
    saved_animation_path            = ROOT / "EBASTv2_train" / "simulated_trained_model" / "paper_results" / f"{case_idx}_sac.gif"
    
    # Animate Simulation
    env.instance.AnimateFleetTrajectory(
            ship_ids=None,  
            show=False,
            block=True,
            mode="quick",
            fig_width=7.0,
            margin_frac=0.08,
            equal_aspect=True,
            interval_ms=20,
            frame_step=10,
            trail_len=50,
            plot_routes=True,
            exclude_target_ships_route=True, 
            plot_waypoints=True,
            plot_roa=True,
            plot_start_end=True,
            plot_inter_wp_roa=False,
            plot_inter_wp_proj=False,
            with_labels=True,
            precompute_ship_outlines=True,
            save_path=saved_animation_path,
            writer_fps=20,
            palette=None,
            blit=True,
            ship_scale=1.0
        )

    # Legend Location
    # +--------------+--------------+---------------+
    # | 'upper left' |'upper center'| 'upper right' |
    # +--------------+--------------+---------------+
    # |'center left' |   'center'   |'center right' |
    # +--------------+--------------+---------------+
    # | 'lower left' |'lower center'| 'lower right' |
    # +--------------+--------------+---------------+
    
    # Plot Trajectory
    saved_figure_path = ROOT / "EBASTv2_train" / "simulated_trained_model" / "paper_results" / f"{case_idx}_sac.pdf"
    # saved_figure_path = None
    # fig_widt = 5.0 for 'quick', 2.3 for 'paper'
    env.instance.PlotFleetTrajectory(
        mode="paper",
        every_n=100, 
        fig_width=3.4,
        exclude_target_ships_route=True,
        disable_title=True,
        plot_IWs=True,
        plot_IW_names=False,
        plot_time_line_connection=False,
        ship_scale=10.0,
        legend_loc='upper right',
        save_path=saved_figure_path,
        show=False,
    )
    
evaluate_failure    = False    
evaluate_failure    = True

if evaluate_failure:
    indices=list(range(100))
    evaluate_failure_cases(env,
        model=sac_model,
        recap_path=recap_path,
        case_indices=indices,
        deterministic=True,
    )