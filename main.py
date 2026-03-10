from pathlib import Path
import sys
import os

# Ensure libcosim DLL is found
dll_dir = Path(sys.prefix) / "Lib" / "site-packages" / "libcosimpy" / "libcosimc"
os.add_dll_directory(str(dll_dir))

## PATH HELPER (OBLIGATORY)
# project root = two levels up from this file
ROOT = Path(__file__).resolve().parents[0]
sys.path.insert(0, str(ROOT))

from orchestrator.sit_cosim import ShipInTransitCoSimulation

# =========================
# Load the Configuration
# =========================
import yaml

## Get the config path

# config_path = ROOT / "config" / "non_ast" / "step_size_5s" / "single_ship_config.yaml"
# config_path = ROOT / "config" / "non_ast" / "step_size_5s" / "single_ship_config_w_env.yaml"
# config_path = ROOT / "config" / "non_ast" / "step_size_5s" / "single_target_ship_config.yaml"
# config_path = ROOT / "config" / "non_ast" / "step_size_5s" / "single_target_ship_config_w_env.yaml"
config_path = ROOT / "config" / "non_ast" / "step_size_5s" / "multi_target_ship_config.yaml"
# config_path = ROOT / "config" / "non_ast" / "step_size_5s" / "multi_target_ship_config_w_env.yaml"

# config_path = ROOT / "config" / "non_ast" / "delayed_start" / "single_ship_delayed_start_test.yaml"
# config_path = ROOT / "config" / "non_ast" / "delayed_start" / "single_target_ship_delayed_start_test.yaml"
# config_path = ROOT / "config" / "non_ast" / "delayed_start" / "multi_target_ship_delayed_start_test.yaml"

## Get the save path for animation

# save_path = ROOT / "saved_animation" / "single_ship.mp4"
# save_path = ROOT / "saved_animation" / "single_ship_w_env.mp4"
# save_path = ROOT / "saved_animation" / "single_target_ship.mp4"
# save_path = ROOT / "saved_animation" / "single_target_ship_w_env.mp4"
# save_path = ROOT / "saved_animation" / "multi_target_ship.mp4"
# save_path = ROOT / "saved_animation" / "multi_target_ship_w_env.mp4"

# save_path = ROOT / "saved_animation" / "single_ship_delayed_start_test.mp4"
# save_path = ROOT / "saved_animation" / "single_target_ship_delayed_start_test.mp4"
save_path = ROOT / "saved_animation" / "multi_target_ship_delayed_start_test.mp4"


with config_path.open("r", encoding="utf-8") as f:
    config = yaml.safe_load(f)
    
# =========================
# Instantiate Co-simulation Wrapper
# =========================
# Instantiate
instance = ShipInTransitCoSimulation(config=config, ROOT=ROOT)


# =========================
# Simulate
# =========================
instance.Simulate()

# =========================
# Animation and Plot
# =========================
# Available formats:
# - .mp4
# - .gif
# - .avi
# - .mov

# Animate Simulation
instance.AnimateFleetTrajectory(
        ship_ids=None,
        fig_width=10.0,
        margin_frac=0.08,
        equal_aspect=True,
        interval_ms=60,
        frame_step=2,
        trail_len=50,
        plot_routes=True,
        plot_waypoints=True,
        plot_roa=True,
        with_labels=True,
        precompute_outlines=True,
        # save_path=save_path,
        writer_fps=60,
        show=True,
        block=True,
        palette=None,
        blit=True
    )

# Plot Trajectory
instance.PlotFleetTrajectory()

# Plot Simulation Results
key_group_list = [
    ## Own Ship
    # Base results
    ["OS0.north"],
    ["OS0.east"],
    ["OS0.forward_speed", "OS0.next_wp_speed", "OS0.total_ship_speed"],
    ["OS0.yaw_angle_rad", "OS0.yaw_angle_ref_rad"],
    ["OS0.rudder_angle_deg"],
    ["OS0.e_ct"],
    ["OS0.shaft_speed_rpm", "OS0.shaft_speed_cmd_rpm"],
    ["OS0.throttle_cmd"],
    
    # For non-single ship simulation only
    ["OS0.new_throttle_cmd"],
    ["OS0.new_rudder_angle_deg"],
    ["OS0.colav_rud_ang_increment"],
    ["OS0.beta_own_to_tar_1"],
    ["OS0.tcpa_own_to_tar_1"],
    ["OS0.dcpa_own_to_tar_1"],
    ["OS0.dist_own_to_tar_1"],
    ["OS0.rr_own_to_tar_1"],
    
    # # For environment load-enabled simulation only
    # ["OS0.current_speed"],
    # ["OS0.current_direction_deg"],
    # ["OS0.wind_speed"],
    # ["OS0.wind_direction_deg"],
    
    ## Target Ship(s)
    # Base results
    ["TS1.north"],
    ["TS1.east"],
    ["TS1.forward_speed", "TS1.next_wp_speed", "TS1.total_ship_speed"],
    ["TS1.yaw_angle_rad", "TS1.yaw_angle_ref_rad"],
    ["TS1.rudder_angle_deg"],
    ["TS1.e_ct"],
    ["TS1.thrust_force"]
]

# Plot Time Series
instance.JoinPlotTimeSeries(list(reversed(key_group_list)),  create_title= False, legend= True, show_instance_name=False, show=True)