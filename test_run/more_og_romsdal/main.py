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

from orchestrator.sit_cosim import ShipInTransitCoSimulation
from orchestrator.sit_cosim_fmpy import ShipInTransitCoSimulation
from orchestrator.scenario_config import load_base_config

# =========================
# Load the Configuration
# =========================
import yaml

## Get the config path
config_path = ROOT / "test_run" / "more_og_romsdal" / "more_og_romsdal.yaml"

## Get the configs
config      = load_base_config(config_path)

# =========================
# Spawn Requests
# =========================
# Spawn requests (More og Romsdal)  
own_ship = {
    "start_time"        : 0.0,
    "speed_setpoints"   : [0, 4, 4, 3, 5, 6, 6, 6, 6]
}

spawn_requests = {
    "OS0": own_ship,
}
    
# =========================
# Instantiate Co-simulation Wrapper
# =========================
# Flag for map evaluation
skip_map_evaluation=False

# Instantiate
instance = ShipInTransitCoSimulation(config=config, ROOT=ROOT, 
                                     spawn_requests=spawn_requests,
                                     skip_map_evaluation=skip_map_evaluation)
# WARNING!
# Setting "skip_map_evaluation" to False enables grounding and outside_map_horizon checking, 
# however this will increase the runtime by A LOT. As default, the value is set to True. 
# Set the value to False when you need it: you WILL know it when you really need it!

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
save_path = ROOT / "saved_animation" / "more_og_romsdal.gif"

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
        save_path=save_path,
        writer_fps=20,
        palette=None,
        blit=True,
        ship_scale=1.0
    )

# # Plot Trajectory
# instance.PlotFleetTrajectory(mode="quick", ship_scale=1.0)

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
    
    # For environment load-enabled simulation only
    ["OS0.current_speed"],
    ["OS0.current_direction_deg"],
    ["OS0.wind_speed"],
    ["OS0.wind_direction_deg"],
]

# # Plot Time Series
# instance.JoinPlotTimeSeries(list(reversed(key_group_list)),  
#                             create_title= False, 
#                             legend= True, 
#                             show_instance_name=False,
#                             show_separately=False,
#                             show=True,
#                             mode="quick")