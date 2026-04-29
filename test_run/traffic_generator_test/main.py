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

from orchestrator.sit_cosim import ShipInTransitCoSimulation
from orchestrator.scenario_config import prepare_config_and_spawn_requests_with_traffic_gen

# =========================
# Load the Configuration
# =========================
import yaml
import random

## Get the config path
config_path = ROOT / "test_run" / "traffic_generator_test" / "traffic_gen.yaml"

## Get the save path for animation
save_path = ROOT / "saved_animation" / "traffic_gen.gif"

## Get the encounter settings path
encounter_settings_path = ROOT / "test_run" / "traffic_generator_test" / "encounter_settings.json"

# with config_path.open("r", encoding="utf-8") as f:
#     config = yaml.safe_load(f)

# =========================
# Alter the Configuration with generated scenario
# =========================
own_ship_initial = {
    "position": {
        "north": 0.0,
        "east": 0.0,
    },
    "sog": 10.0,
    "cog": 0.0,
    "heading": 1.0,
    "navStatus": "Under way using engine",
}

availableEncounterTypes = ["head-on", 
                        #    "overtaking-give-way",
                        #    "overtaking-stand-on",
                        #    "crossing-give-way",
                           "crossing-stand-on"]

random.seed(42)

encounter_type_TS1 = random.choice(availableEncounterTypes)
encounter_type_TS2 = random.choice(availableEncounterTypes)

print(f"Target Ship 1 encounter type: {encounter_type_TS1}")
print(f"Target Ship 2 encounter type: {encounter_type_TS2}")

encounters = {
    "TS1" : {
        "desiredEncounterType": encounter_type_TS1,
        "vectorTime": 25.0,
        "beta": 2.0,
        "relativeSpeed": 1.2,
        },
    "TS2" : {
        "desiredEncounterType":  encounter_type_TS2,
        "vectorTime": 25.0,
        "beta": -3.0,
        "relativeSpeed": 1.2,
        },
}

config, spawn_requests = prepare_config_and_spawn_requests_with_traffic_gen(own_ship_initial, 
                                                                            encounters, 
                                                                            config_path, 
                                                                            encounter_settings_path)

# =========================
# Instantiate Co-simulation Wrapper
# =========================
# Instantiate
instance = ShipInTransitCoSimulation(config=config, 
                                     spawn_requests=spawn_requests, 
                                     ROOT=ROOT)


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
        show=True,
        block=True,
        mode="quick",
        fig_width=10.0,
        margin_frac=0.08,
        equal_aspect=True,
        interval_ms=20,
        frame_step=2,
        trail_len=50,
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
    ["OS0.north"],
    ["OS0.east"],
    ["OS0.forward_speed", "OS0.next_wp_speed", "OS0.total_ship_speed"],
    ["OS0.yaw_angle_rad", "OS0.yaw_angle_ref_rad"],
    ["OS0.rudder_angle_deg"],
    ["OS0.e_ct"],
    ["OS0.shaft_speed_rpm", "OS0.shaft_speed_cmd_rpm"],
    ["OS0.throttle_cmd"],
    
    # # For non-single ship simulation only
    # ["OS0.new_throttle_cmd"],
    # ["OS0.new_rudder_angle_deg"],
    # ["OS0.colav_rud_ang_increment"],
    # ["OS0.beta_own_to_tar_1"],
    # ["OS0.tcpa_own_to_tar_1"],
    # ["OS0.dcpa_own_to_tar_1"],
    # ["OS0.dist_own_to_tar_1"],
    # ["OS0.rr_own_to_tar_1"],
    
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