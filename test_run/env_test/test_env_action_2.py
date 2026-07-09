from pathlib import Path
import sys
import os

# Ensure libcosim DLL is found
dll_dir = Path(sys.prefix) / "Lib" / "site-packages" / "libcosimpy" / "libcosimc"
os.add_dll_directory(str(dll_dir))

## PATH HELPER (OBLIGATORY)
# project root = two levels up from this file
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from EBASTv2_core.env import EBASTv2Env
from EBASTv2_core.episode_logger import log_episode_recap
from orchestrator.scenario_config import generate_spawn_request_bank, load_spawn_requests_bank_path

import numpy as np

# =========================
# Load the Configuration
# =========================
# Get the config path
config_path = ROOT / "test_run" / "env_test" / "test_env_2.yaml"

# Get the save path for animation
save_path = ROOT / "saved_animation" / "test_env_action_ts_2_RF.mp4"

# Get the encounter settings path
encounter_settings_path = ROOT / "test_run" / "env_test" / "encounter_settings.json"

# Log path
log_path = ROOT / "test_run" / "env_test" / "logs" / "episode_recap.txt"

# Spawn requests bank path
spawn_requests_bank_path = ROOT / "test_run" / "env_test" / "spawn_request_bank.pkl"

# =========================
# Instantiate the environment wrapper
# =========================
# Generates/collect spawn requests
spawn_requests_bank_path    = generate_spawn_request_bank(ROOT=ROOT,
                                                          config_path=config_path,
                                                          encounter_settings_path=encounter_settings_path,
                                                          spawn_requests_bank_path=spawn_requests_bank_path,
                                                          n_cases=100,
                                                          training_case_ratio=0.8,                              # Specifically for RL-env
                                                          overwrite=False)
spawn_requests_bank         = load_spawn_requests_bank_path(spawn_requests_bank_path)

# Instantiate the RL-environment wrapper class
env = EBASTv2Env(
    ROOT=ROOT,
    config_path=config_path,
    encounter_settings_path=encounter_settings_path,
    spawn_requests_bank=spawn_requests_bank,
    use_fmpy=True
    )

env.set_for_evaluation()

case_idx    = 82
obs, info   = env.reset(seed=250, specific_case_idx=case_idx)

action_list_1 = [[25,3500],
                 [25,3500],
                 [25,3500],
                 [25,3500],
                 [25,3500]]

action_list_2 = [[-13,500],
                 [-13,2500],
                 [-13,2500],
                 [-15,2500],
                 [-15,2500]]

term = False
i_ts1   = 0
i_ts2   = 0

while not term:
    obs                 = env._get_obs()
    action_masks        = obs["action_masks"]
    do_action_sampling  = any(action_masks)
    
    if do_action_sampling:
        action = [[0, 1000]] * len(action_masks)
        
        if bool(action_masks[0]) is True:
            action[0]   = (action_list_1[i_ts1])
            i_ts1      += 1
        if bool(action_masks[1]) is True:
            action[1]   = action_list_2[i_ts2]
            i_ts2      += 1
        
        action_flatten  = env._normalize_action(np.array(action).flatten())
        
        obs, rew, term, trun, inf = env.step(action_flatten)

# =========================
# Log the episode
# =========================
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
        plot_inter_wp_roa=True,
        plot_inter_wp_proj=False,
        with_labels=True,
        precompute_ship_outlines=True,
        # save_path=save_path,
        writer_fps=20,
        palette=None,
        blit=True,
        ship_scale=1.0
    )

# Plot Trajectory
env.instance.PlotFleetTrajectory(mode="quick", ship_scale=1.0)