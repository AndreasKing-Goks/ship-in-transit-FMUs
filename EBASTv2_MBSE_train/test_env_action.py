from pathlib import Path
import sys
import os

# Ensure libcosim DLL is found
dll_dir = Path(sys.prefix) / "Lib" / "site-packages" / "libcosimpy" / "libcosimc"
os.add_dll_directory(str(dll_dir))

## PATH HELPER (OBLIGATORY)
# project root = two levels up from this file
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from EBASTv2_MBSE_train.env_EBASTv2_MBSE import EBASTv2EnvMBSE
from EBASTv2_core.episode_logger import log_episode_recap
from orchestrator.scenario_config import generate_spawn_request_bank, load_spawn_requests_bank_path

from itertools import product
import numpy as np

# =========================
# Load the Configuration
# =========================
# Get the config path
config_path = ROOT / "EBASTv2_MBSE_train" / "EBASTv2_MBSE.yaml",

# Get the save path for animation
save_path = ROOT / "EBASTv2_MBSE_train" /  "saved_animation" / "EBASTv2MBSE_test.mp4"

# Log path
log_path = ROOT / "EBASTv2_MBSE_train" / "logs" / "EBASTv2MBSE_episode_recap.txt"

# =========================
# Instantiate the environment wrapper
# =========================
## Define custom spawn requests bank
# Maximum distance of the frame
max_dist = 10_000


# Common own ship configuration
own_ship = {
    "start_time": 0.0,
    "north_route": [0, max_dist],
    "east_route": [0, max_dist],
    "speed_setpoints": 5.0,
}

# Target ship routes for each case [(north_routes, east_routes)]
target_routes = [
    ([0, max_dist], [max_dist, 0]),  # Case 0: OS0 Give Way
    ([max_dist, 0], [0, max_dist]),  # Case 1: OS0 Stand On
]
target_start_times  = [0.0, 300.0, 600.0]

# Generate cases
cases = [
    {
        "case_id": i,
        "spawn_requests": {
            "OS0": own_ship,
            "TS1": {
                "start_time": start_time,
                "north_route": north,
                "east_route": east,
                "speed_setpoints": 5.0,
            },
        },
        "own_ship_initial": None,
        "encounters": None,
    }
    for i, ((north, east), start_time) in enumerate(product(target_routes, target_start_times))
]

# Spawn request bank
margin  = 1000
rounded_north_bound = [-margin, max_dist+margin]
rounded_east_bound  = [-margin, max_dist+margin]

spawn_requests_bank = {
    "path": None,
    "n_cases": len(cases),
    "start_eval_case_id": 0,
    "cases": cases,
    "rounded_north_bound": rounded_north_bound,
    "rounded_east_bound": rounded_east_bound,
}


# Instantiate the RL-environment wrapper class
env = EBASTv2EnvMBSE(
    ROOT=ROOT,
    config_path=config_path,
    spawn_requests_bank=spawn_requests_bank,
    detailed_reward=True
    )

env.set_for_evaluation()

case_idx    = 2
obs, info   = env.reset(seed=250, specific_case_idx=case_idx)

action_list = [
    [13,2500],
    [13,2500],
    [13,2500],
    [23,2500],
    [23,2500]
    ] # Collision

n_sample = len(action_list)

i = 0
term = False
while not term:
    obs = env._get_obs()
    do_action_sampling = any(obs["action_masks"])
    if do_action_sampling:
        action  = env._normalize_action(action_list[i])
        obs, rew, term, trun, inf = env.step(action)
        i += 1

# # =========================
# # Log the episode
# # =========================
# log_episode_recap(env=env, log_path=log_path)
# print(f"Episode recap saved to: {log_path}")

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

# # Plot Trajectory
# env.instance.PlotFleetTrajectory(mode="quick", ship_scale=1.0)

# # Plot Simulation Results
# key_group_list = [
#     ## Own Ship
#     # Base results
#     ["OS0.north"],
#     ["OS0.east"],
#     ["OS0.forward_speed", "OS0.next_wp_speed", "OS0.total_ship_speed"],
#     ["OS0.yaw_angle_rad", "OS0.yaw_angle_ref_rad"],
#     ["OS0.rudder_angle_deg"],
#     ["OS0.e_ct"],
#     ["OS0.shaft_speed_rpm", "OS0.shaft_speed_cmd_rpm"],
#     ["OS0.throttle_cmd"],
    
#     ## Target Ship(s)
#     # Base results
#     ["TS2.north"],
#     ["TS2.east"],
#     ["TS2.forward_speed", "TS2.next_wp_speed", "TS2.total_ship_speed"],
#     ["TS2.yaw_angle_rad", "TS2.yaw_angle_ref_rad"],
#     ["TS2.rudder_angle_deg"],
#     ["TS2.e_ct"],
#     ["TS2.thrust_force"]
# ]

# # Plot Time Series
# # env.instance.JoinPlotTimeSeries(list(reversed(key_group_list)),  create_title= False, legend= True, show_instance_name=False, show=True)