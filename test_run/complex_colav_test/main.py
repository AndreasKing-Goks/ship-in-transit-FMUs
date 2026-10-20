from pathlib import Path
import sys
import os
import time

# Ensure libcosim DLL is found
dll_dir = Path(sys.prefix) / "Lib" / "site-packages" / "libcosimpy" / "libcosimc"
os.add_dll_directory(str(dll_dir))

## PATH HELPER (OBLIGATORY)
# project root = two levels up from this file
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

# from orchestrator.sit_cosim import ShipInTransitCoSimulation
from orchestrator.complex_sit_cosim_fmpy import ShipInTransitCoSimulation
from orchestrator.scenario_config import load_base_config

# =========================
# Load the Configuration
# =========================
import yaml

## Get the config path
config_path = ROOT / "test_run" / "complex_colav_test" / "complex_colav_test.yaml"

## Get the configs
config      = load_base_config(config_path)
    
# =========================
# Instantiate Co-simulation Wrapper
# =========================
# Flag for map evaluation
skip_map_evaluation=False

# Instantiate
instance = ShipInTransitCoSimulation(config=config, ROOT=ROOT, 
                                     skip_map_evaluation=skip_map_evaluation)

# =========================
# Simulate
# =========================
start_time = time.time()
instance.Simulate()
time_count = time.time() -start_time
print(f"Skip Map evaluation: {skip_map_evaluation}")
print(f"A single simulation finished in {time_count:.2f} seconds")

# =========================
# Animation and Plot
# =========================
# Available formats:
# - .mp4
# - .gif
# - .avi
# - .mov

## Get the save path for animation
save_path = ROOT / "saved_animation" / "complex_colav_test.gif"

# Animate Simulation
instance.AnimateFleetTrajectory(
        ship_ids=None,
        show=True,
        block=True,
        mode="quick",
        fig_width=10.0,
        margin_frac=0.08,
        equal_aspect=True,
        interval_ms=60,
        frame_step=5,
        trail_len=300,
        plot_routes=True,
        plot_waypoints=True,
        plot_roa=True,
        plot_start_end=True,
        with_labels=True,
        precompute_ship_outlines=True,
        # save_path=save_path,
        writer_fps=20,
        palette=None,
        blit=True,
        ship_scale=1.0
    )

# Plot Trajectory
instance.PlotFleetTrajectory(mode="quick", ship_scale=1.0)

# Plot Simulation Results
key_group_list = [
    ## Own Ship
    # Base results
    ["OS0.SHIP_MODEL.north", "OS0.SHIP_EKF.estimated_n"],
    ["OS0.SHIP_MODEL.east", "OS0.SHIP_EKF.estimated_e"],
    ["OS0.SHIP_MODEL.forward_speed", "OS0.MISSION_MANAGER.next_wp_speed", "OS0.SHIP_MODEL.total_ship_speed"],
    ["OS0.SHIP_MODEL.yaw_angle_rad", "OS0.AUTOPILOT.yaw_angle_ref_rad", "OS0.SHIP_EKF.estimated_psi"],
    ["OS0.AUTOPILOT.rudder_angle_deg"],
    ["OS0.AUTOPILOT.e_ct"],
    ["OS0.MACHINERY_SYSTEM.shaft_speed_rpm", "OS0.SHAFT_SPEED_CONTROLLER.shaft_speed_cmd_rpm"],
    ["OS0.THROTTLE_CONTROLLER.throttle_cmd"],
    
    # # For environment load-enabled simulation only
    # ["OS0.SURFACE_CURRENT.current_speed"],
    # ["OS0.SURFACE_CURRENT.current_direction_deg"],
    # ["OS0.WIND.wind_speed"],
    # ["OS0.WIND.wind_direction_deg"],
    
    # OS COLAV
    ["OS0.COLAV.colav_speed_reduction_factor"],
    ["OS0.COLAV.colav_heading_offset_deg"],
    ["OS0.COLAV.risk_score_tar_1"],
    ["OS0.COLAV.encounter_wrt_tar_1"],
    ["OS0.COLAV.role_wrt_tar_1"],
    ["OS0.COLAV.beta_own_to_tar_1"],
    ["OS0.COLAV.beta_tar_to_own_1"],
    ["OS0.COLAV.rr_own_to_tar_1"],
    
    # # TS COLAV
    # ["TS1.COLAV.colav_speed_reduction_factor"],
    # ["TS1.COLAV.colav_heading_offset_deg"],
    # ["TS1.COLAV.risk_score_tar_1"],
    # ["TS1.COLAV.encounter_wrt_tar_1"],
    # ["TS1.COLAV.role_wrt_tar_1"],
    # ["TS1.COLAV.beta_own_to_tar_1"],
    # ["TS1.COLAV.beta_tar_to_own_1"],
    # ["TS1.COLAV.rr_own_to_tar_1"],
    
    # # For Ship EKF
    # ["OS0.SHIP_EKF.estimated_n"],
    # ["OS0.SHIP_EKF.estimated_e"],
    # ["OS0.SHIP_EKF.estimated_psi"],
    # ["OS0.SHIP_EKF.estimated_u"],
    # ["OS0.SHIP_EKF.estimated_v"],
    # ["OS0.SHIP_EKF.estimated_r"],
    
    # # Sensors
    # ["OS0.GPS.measured_north"],
    # ["OS0.GPS.measured_east"],
    # ["OS0.GYROCOMPASS.measured_ship_heading_rad"],
    # ["OS0.SPEED_LOG.measured_ship_speed"],
    # ["OS0.RADAR.tar_1_range"],
    # ["OS0.RADAR.tar_1_range_rate"],
    # ["OS0.RADAR.tar_1_bearing"],
    # ["OS0.CAMERA_FORE.tar_1_range"],
    # ["OS0.CAMERA_FORE.tar_1_bearing"],
    # ["OS0.CAMERA_AFT.tar_1_range"],
    # ["OS0.CAMERA_AFT.tar_1_bearing"],
    # ["OS0.CAMERA_PORT.tar_1_range"],
    # ["OS0.CAMERA_PORT.tar_1_bearing"],
    # ["OS0.CAMERA_STARBOARD.tar_1_range"],
    # ["OS0.CAMERA_STARBOARD.tar_1_bearing"],
    
    ## Target Ship(s)
    # Base results
    ["TS1.SHIP_MODEL.north", "OS0.TS_TRACKING_EKF_TS1.estimated_n"],
    ["TS1.SHIP_MODEL.east", "OS0.TS_TRACKING_EKF_TS1.estimated_e"],
    ["TS1.SHIP_MODEL.forward_speed", "TS1.MISSION_MANAGER.next_wp_speed", "TS1.SHIP_MODEL.total_ship_speed"],
    ["TS1.SHIP_MODEL.yaw_angle_rad", "TS1.AUTOPILOT.yaw_angle_ref_rad"],
    ["TS1.AUTOPILOT.rudder_angle_deg"],
    ["TS1.AUTOPILOT.e_ct"],
    ["TS1.SPEED_CONTROLLER.thrust_force"],
    
    # ["TS1.MISSION_MANAGER.prev_wp_north"],
    # ["TS1.MISSION_MANAGER.prev_wp_east"],
    # ["TS1.MISSION_MANAGER.next_wp_north"],
    # ["TS1.MISSION_MANAGER.next_wp_east"],
    # ["TS1.AUTOPILOT.yaw_angle_ref_rad"],
]

# Plot Time Series
instance.JoinPlotTimeSeries(list(reversed(key_group_list)),  
                            create_title= False, 
                            legend= True, 
                            show_instance_name=False,
                            show_separately=False,
                            show=True,
                            mode="quick")