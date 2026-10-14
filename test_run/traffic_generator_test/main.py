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
from orchestrator.scenario_config import (prepare_spawn_requests_with_traffic_gen,
                                          sample_beta_and_rel_speed_given_encounter_settings,
                                          load_base_config)

# =========================
# Load the Configuration
# =========================
import random
import numpy as np

## Get the config path
config_path = ROOT / "test_run" / "traffic_generator_test" / "traffic_gen.yaml"

## Get the save path for animation
save_path = ROOT / "saved_animation" / "traffic_gen.gif"

## Get the encounter settings path
encounter_settings_path = ROOT / "test_run" / "traffic_generator_test" / "encounter_settings.json"

# =========================
# Alter the Configuration with generated scenario
# =========================
availableNavStatus = ["Under way using engine",
                      "At anchor",
                      "Not under command",
                      "Restricted maneuverability",
                      "Constrained by draft",
                      "Moored",
                      "Aground",
                      "Engaged in fishing",
                      "Under way sailing",
                      "Reserved for future use"
                      ]

availableEncounterTypes = ["head-on", 
                           "overtaking-give-way",
                           "overtaking-stand-on",
                           "crossing-give-way",
                           "crossing-stand-on"
                           ]

initial_own_ship_cog_and_heading    = np.arange(0.0, 360.0, 15.0)
os_init_heading                     = random.choice(initial_own_ship_cog_and_heading)

own_ship_initial = {
    "position": {
        "north": 0.0,
        "east": 0.0,
    },
    "sog": 6.0,    # m/s
    "cog": os_init_heading,
    "heading": os_init_heading,
    "navStatus": "Under way using engine",
}

# seed = 42
# random.seed(seed)

encounter_type_TS1 = random.choice(availableEncounterTypes)
encounter_type_TS2 = random.choice(availableEncounterTypes)

print(f"Target Ship 1 encounter type: {encounter_type_TS1}")
print(f"Target Ship 2 encounter type: {encounter_type_TS2}")

# beta_ts1, rel_speed_ts1 = sample_beta_and_rel_speed_given_encounter_settings(encounter_type_TS1,
#                                                                              encounter_settings_path,
#                                                                              seed=seed+1)
# beta_ts2, rel_speed_ts2 = sample_beta_and_rel_speed_given_encounter_settings(encounter_type_TS2,
#                                                                              encounter_settings_path,
#                                                                              seed=seed+2)

beta_ts1, rel_speed_ts1 = sample_beta_and_rel_speed_given_encounter_settings(encounter_type_TS1,
                                                                             encounter_settings_path)
beta_ts2, rel_speed_ts2 = sample_beta_and_rel_speed_given_encounter_settings(encounter_type_TS2,
                                                                             encounter_settings_path)

encounters = {
    "TS1" : {
        "desiredEncounterType": encounter_type_TS1,
        "vectorTime": 10.0,
        "beta": beta_ts1,
        "relativeSpeed": rel_speed_ts1,
        },
    "TS2" : {
        "desiredEncounterType": encounter_type_TS2,
        "vectorTime": 15.0,
        "beta": beta_ts2,
        "relativeSpeed": rel_speed_ts2,
        },
}

config          = load_base_config(config_path)
spawn_requests  = prepare_spawn_requests_with_traffic_gen(own_ship_initial, 
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
        frame_step=10,
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