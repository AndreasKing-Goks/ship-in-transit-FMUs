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

from EBASTv2_MBSE_train.env_EBASTv2_MBSE import EBASTv2EnvMBSE
from EBASTv2_MBSE_train.MBSE_episode_logger import log_episode_recap, log_training_args
from EBASTv2_train.evaluate_failure_cases import evaluate_failure_cases
from EBASTv2_core.path_utils import get_RL_model_path

import numpy as np
from itertools import product

# =========================
# Handle paths
# =========================
# Trained Model Name
model_name                      = "EB-ASTv2_MBSE_2026-10-23_21-11-07_b58a"

# Get the config path
config_path = [
        ROOT / "EBASTv2_MBSE_train" / "EBASTv2_MBSE.yaml",
    ]

# Get the trained model
model_path                      = ROOT / "EBASTv2_MBSE_train" / "trained_model" / model_name / "model" / "model.zip"

# Evaluation recap
recap_path                      = ROOT / "EBASTv2_MBSE_train" / "simulated_trained_model" / "evaluation_recap_EBASTv2_MBSE.txt"

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
target_start_times  = [0.0, 150, 300.0]

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
    "start_eval_case_id": len(cases),
    "cases": cases,
    "north_bound": rounded_north_bound,
    "east_bound": rounded_east_bound,
    "rounded_north_bound": rounded_north_bound,
    "rounded_east_bound": rounded_east_bound,
}

# Instantiate the RL-environment wrapper class
env = EBASTv2EnvMBSE(
    ROOT=ROOT,
    config_path=config_path,
    spawn_requests_bank=spawn_requests_bank,
    use_fmpy=True
    )

# =========================
# Load the trained model
# =========================
# Load the trained model
recurrent_ppo_model = RecurrentPPO.load(model_path)

# =========================
# Run the trained model and log the episode
# =========================
simulate    = False
simulate    = True

if simulate:
    # Set the environment to evaluation mode
    # env.set_for_evaluation()
    
    # Reset the trained model
    case_idx    = 5
    config_idx  = 0
    seed        = None
    obs, _      = env.reset(seed=seed, specific_case_idx=case_idx, specific_config_idx=config_idx)

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
        done = bool(terminated) or bool(truncated)

        # Keep shape (1,), rather than changing to scalar bool
        episode_starts = np.array([done], dtype=bool)
        
        # Break the loop if it's either terminated or truncated
        if done:
            break
    
    # # Log path
    # log_path                        = ROOT / "EBASTv2_MBSE_train" / "simulated_trained_model" / f"{case_idx}_episode_recap_EBASTv2_MBSE.txt"
    
    # # Log the episodes
    # log_episode_recap(env=env, log_path=log_path)
    # print(f"Episode recap saved to: {log_path}")

    # # =========================
    # # Animation and Plot
    # # =========================
    # # Available formats:
    # # - .mp4
    # # - .gif
    # # - .avi
    # # - .mov
    
    # # Get the save path for animation
    # saved_animation_path            = ROOT / "EBASTv2_MBSE_train" / "simulated_trained_model" / "paper_results" / f"{case_idx}_EBASTv2_MBSE.gif"

    # # Animate Simulation
    # env.instance.AnimateFleetTrajectory(
    #         ship_ids=None,  
    #         show=True,
    #         block=True,
    #         mode="quick",
    #         fig_width=7.0,
    #         margin_frac=0.08,
    #         equal_aspect=True,
    #         interval_ms=20,
    #         frame_step=10,
    #         trail_len=50,
    #         plot_routes=True,
    #         exclude_target_ships_route=True, 
    #         plot_waypoints=True,
    #         plot_roa=True,
    #         plot_start_end=True,
    #         plot_inter_wp_roa=False,
    #         plot_inter_wp_proj=False,
    #         with_labels=True,
    #         precompute_ship_outlines=True,
    #         save_path=saved_animation_path,
    #         writer_fps=20,
    #         palette=None,
    #         blit=True,
    #         ship_scale=1.0
    #     )

    # # Legend Location
    # # +--------------+--------------+---------------+
    # # | 'upper left' |'upper center'| 'upper right' |
    # # +--------------+--------------+---------------+
    # # |'center left' |   'center'   |'center right' |
    # # +--------------+--------------+---------------+
    # # | 'lower left' |'lower center'| 'lower right' |
    # # +--------------+--------------+---------------+
    
    # Plot Trajectory
    saved_figure_path = ROOT / "EBASTv2_MBSE_train" / "simulated_trained_model" / "paper_results" / f"{case_idx}_EBASTv2_MBSE.jpg"
    # saved_figure_path = None
    # fig_width = 5.0 for 'quick', 3.4 for 'paper'
    env.instance.PlotFleetTrajectory(
        mode="paper",
        every_n=100, 
        fig_width=3.4,
        exclude_target_ships_route=True,
        disable_title=True,
        plot_IWs=True,
        plot_IW_names=False,
        plot_time_line_connection=True,
        ship_scale=10.0,
        legend_loc='center right',
        save_path=saved_figure_path,
        show=False,
    )

    plot_simulation_results = False
    plot_simulation_results = True

    if plot_simulation_results:
        save_dir_path   = ROOT / "EBASTv2_MBSE_train" / "simulated_trained_model" / "simulation_results_plot"
        
        key_group_list  = [
            ## Own Ship
            # Base results
            (["OS0.SHIP_MODEL.north", "OS0.SHIP_EKF.estimated_n"], f"{case_idx}_OS0_north.jpg"),
            (["OS0.SHIP_MODEL.east", "OS0.SHIP_EKF.estimated_e"], f"{case_idx}_OS0_east.jpg"),
            (["OS0.SHIP_MODEL.forward_speed", "OS0.MISSION_MANAGER.next_wp_speed", "OS0.SHIP_MODEL.total_ship_speed"], f"{case_idx}_OS0_speed.jpg"),
            (["OS0.SHIP_MODEL.yaw_angle_rad", "OS0.AUTOPILOT.yaw_angle_ref_rad", "OS0.SHIP_EKF.estimated_psi"], f"{case_idx}_OS0_heading.jpg"),
            (["OS0.AUTOPILOT.rudder_angle_deg"], f"{case_idx}_OS0_rud_angle.jpg"),
            (["OS0.AUTOPILOT.e_ct"], f"{case_idx}_OS0_ect.jpg"),
            (["OS0.MACHINERY_SYSTEM.shaft_speed_rpm", "OS0.SHAFT_SPEED_CONTROLLER.shaft_speed_cmd_rpm"], f"{case_idx}_OS0_shaft_speed.jpg"),
            (["OS0.THROTTLE_CONTROLLER.throttle_cmd"], f"{case_idx}_OS0_throttle.jpg"),
            
            # # For environment load-enabled simulation only
            # (["OS0.SURFACE_CURRENT.current_speed"], f"{case_idx}_OS0_current_speed.jpg"),
            # (["OS0.SURFACE_CURRENT.current_direction_deg"], f"{case_idx}_OS0_current_dir.jpg"),
            # (["OS0.WIND.wind_speed"], f"{case_idx}_OS0_wind_speed.jpg"),
            # (["OS0.WIND.wind_direction_deg"], f"{case_idx}_OS0_wind_dir.jpg"),
            
            # OS COLAV
            (["OS0.COLAV.colav_speed_reduction_factor"], f"{case_idx}_OS0_csrf.jpg"),
            (["OS0.COLAV.colav_heading_offset_deg"], f"{case_idx}_OS0_chod.jpg"),
            (["OS0.COLAV.risk_score_tar_1"], f"{case_idx}_OS0_rst_1.jpg"),
            (["OS0.COLAV.encounter_wrt_tar_1"], f"{case_idx}_OS0_enc_t1.jpg"),
            (["OS0.COLAV.role_wrt_tar_1"], f"{case_idx}_OS0_rol_t1.jpg"),
            # (["OS0.COLAV.beta_own_to_tar_1"], f"{case_idx}_OS0_beta_t1.jpg"),
            # (["OS0.COLAV.beta_tar_to_own_1"], f"{case_idx}_OS0_beta_o1.jpg"),
            # (["OS0.COLAV.rr_own_to_tar_1"], f"{case_idx}_OS0_rr_t1.jpg"),
            
            # # TS COLAV
            # (["TS1.COLAV.colav_speed_reduction_factor"], f"{case_idx}_TS1_csrf.jpg"),
            # (["TS1.COLAV.colav_heading_offset_deg"], f"{case_idx}_TS1_chod.jpg"),
            # (["TS1.COLAV.risk_score_tar_1"], f"{case_idx}_TS1_rst_t1.jpg"),
            # (["TS1.COLAV.encounter_wrt_tar_1"], f"{case_idx}_TS1_enc_t1.jpg"),
            # (["TS1.COLAV.role_wrt_tar_1"], f"{case_idx}_TS1_rol_t1.jpg"),
            # (["TS1.COLAV.beta_own_to_tar_1"], f"{case_idx}_TS1_beta_t1.jpg"),
            # (["TS1.COLAV.beta_tar_to_own_1"], f"{case_idx}_TS1_beta_o1.jpg"),
            # (["TS1.COLAV.rr_own_to_tar_1"], f"{case_idx}_TS1_rr_t1.jpg"),
            
            # # Sensors
            # (["OS0.GPS.measured_north"], f"{case_idx}_OS0_gps_north.jpg"),
            # (["OS0.GPS.measured_east"], f"{case_idx}_OS0_gps_east.jpg"),
            # (["OS0.GYROCOMPASS.measured_ship_heading_rad"], f"{case_idx}_OS0_gyro_heading.jpg"),
            # (["OS0.SPEED_LOG.measured_ship_speed"], f"{case_idx}_OS0_speed_log_speed.jpg"),
            # (["OS0.RADAR.tar_1_range"], f"{case_idx}_OS0_radar_t1_range.jpg"),
            # (["OS0.RADAR.tar_1_range_rate"], f"{case_idx}_OS0_radar_t1_range_rate.jpg"),
            # (["OS0.RADAR.tar_1_bearing"], f"{case_idx}_OS0_radar_t1_beta.jpg"),
            # (["OS0.CAMERA_FORE.tar_1_range"], f"{case_idx}_OS0_camf_range_t1.jpg"),
            # (["OS0.CAMERA_FORE.tar_1_bearing"], f"{case_idx}_OS0_camf_beta_t1.jpg"),
            # (["OS0.CAMERA_AFT.tar_1_range"], f"{case_idx}_OS0_cama_range_t1.jpg"),
            # (["OS0.CAMERA_AFT.tar_1_bearing"], f"{case_idx}_.OS0_cama_beta_t1.jpg"),
            # (["OS0.CAMERA_PORT.tar_1_range"], f"{case_idx}_OS0_camp_range_t1.jpg"),
            # (["OS0.CAMERA_PORT.tar_1_bearing"], f"{case_idx}_OS0_camp_beta_t1.jpg"),
            # (["OS0.CAMERA_STARBOARD.tar_1_range"], f"{case_idx}_OS0_cams_range_t1.jpg"),
            # (["OS0.CAMERA_STARBOARD.tar_1_bearing"], f"{case_idx}_OS0_cams_beta_t1.jpg"),
            ([
                "OS0.TS_TRACKING_EKF_TS1.RADAR_valid_detection",
                "OS0.TS_TRACKING_EKF_TS1.CAMERA_FORE_valid_detection",
                "OS0.TS_TRACKING_EKF_TS1.CAMERA_AFT_valid_detection",
                "OS0.TS_TRACKING_EKF_TS1.CAMERA_PORT_valid_detection",
                "OS0.TS_TRACKING_EKF_TS1.CAMERA_STARBOARD_valid_detection",
                ],
             f"{case_idx}_OS0_sensors_detection_t1.jpg"),
            
            ## Target Ship(s)
            # Base results
            (["TS1.SHIP_MODEL.north", "OS0.TS_TRACKING_EKF_TS1.estimated_n"], f"{case_idx}_TS1_north.jpg"),
            (["TS1.SHIP_MODEL.east", "OS0.TS_TRACKING_EKF_TS1.estimated_e"], f"{case_idx}_TS1_east.jpg"),
            (["TS1.SHIP_MODEL.forward_speed", "TS1.MISSION_MANAGER.next_wp_speed", "TS1.SHIP_MODEL.total_ship_speed"], f"{case_idx}_TS1_speed.jpg"),
            (["TS1.SHIP_MODEL.yaw_angle_rad", "TS1.AUTOPILOT.yaw_angle_ref_rad"], f"{case_idx}_TS1_heading.jpg"),
            (["TS1.AUTOPILOT.rudder_angle_deg"], f"{case_idx}_TS1_rud_ang.jpg"),
            (["TS1.AUTOPILOT.e_ct"], f"{case_idx}_TS1_ect.jpg"),
            (["TS1.SPEED_CONTROLLER.thrust_force"], f"{case_idx}_TS1_throttle.jpg"),
        ]
        
        # Plot Time Series
        env.instance.SaveJoinPlotTimeSeries(list(reversed(key_group_list)), 
                                    save_dir_path=save_dir_path,
                                    case_idx=case_idx,
                                    create_title= False, 
                                    legend= True, 
                                    show_instance_name=False,
                                    show_separately=False,
                                    show=False,
                                    mode="paper")

evaluate_failure    = False    
# evaluate_failure    = True

if evaluate_failure:
    indices=list(range(100))
    evaluate_failure_cases(env,
        model=recurrent_ppo_model,
        recap_path=recap_path,
        case_indices=indices,
        deterministic=True,
    )