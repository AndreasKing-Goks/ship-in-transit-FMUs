"""
Minimal example: run the SAME trafficgen-generated scenario on both Ship-in-Transit
co-simulation backends - libcosimpy (orchestrator/sit_cosim.py) and fmpy
(orchestrator/sit_cosim_fmpy.py) - then check that the two backends produce the same
ship trajectories and compare how long each one took to solve the scenario.

Reuses the trafficgen example config from test_run/traffic_generator_test/.
"""

from pathlib import Path
import sys
import os
import copy
import time

import numpy as np

# Ensure libcosim DLL is found (only needed for the libcosimpy backend)
dll_dir = Path(sys.prefix) / "Lib" / "site-packages" / "libcosimpy" / "libcosimc"
os.add_dll_directory(str(dll_dir))

## PATH HELPER (OBLIGATORY)
# project root = two levels up from this file
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from orchestrator.sit_cosim import ShipInTransitCoSimulation as ShipInTransitCoSimulationLibcosimpy
from orchestrator.sit_cosim_fmpy import ShipInTransitCoSimulation as ShipInTransitCoSimulationFmpy
from orchestrator.scenario_config import (load_base_config,
                                          prepare_spawn_requests_with_traffic_gen,
                                          sample_beta_and_rel_speed_given_encounter_settings)

# =========================
# Generate ONE bank of traffic generator outputs - both backends are run against
# this exact same spawn_requests
# =========================
config_path             = ROOT / "test_run" / "traffic_generator_test" / "traffic_gen.yaml"
encounter_settings_path = ROOT / "test_run" / "traffic_generator_test" / "encounter_settings.json"

own_ship_initial = {
    "position": {"north": 0.0, "east": 0.0},
    "sog": 10.0,
    "cog": 0.0,
    "heading": 0.0,
    "navStatus": "Under way using engine",
}

encounter_type_TS1 = "head-on"
encounter_type_TS2 = "crossing-give-way"

beta_ts1, rel_speed_ts1 = sample_beta_and_rel_speed_given_encounter_settings(encounter_type_TS1, encounter_settings_path)
beta_ts2, rel_speed_ts2 = sample_beta_and_rel_speed_given_encounter_settings(encounter_type_TS2, encounter_settings_path)

encounters = {
    "TS1": {"desiredEncounterType": encounter_type_TS1, "vectorTime": 10.0, "beta": beta_ts1, "relativeSpeed": rel_speed_ts1},
    "TS2": {"desiredEncounterType": encounter_type_TS2, "vectorTime": 15.0, "beta": beta_ts2, "relativeSpeed": rel_speed_ts2},
}

config         = load_base_config(config_path)
spawn_requests = prepare_spawn_requests_with_traffic_gen(own_ship_initial, encounters, config_path, encounter_settings_path)

ship_ids = [ship_config["id"] for ship_config in config["ships"]]

# =========================
# Run the SAME scenario on both backends, timing each one
# (deepcopy config per backend - ShipInTransitCoSimulation mutates it in place)
# =========================
print("Running on libcosimpy backend...")
t0 = time.perf_counter()
instance_libcosimpy = ShipInTransitCoSimulationLibcosimpy(config=copy.deepcopy(config), spawn_requests=spawn_requests, ROOT=ROOT)
instance_libcosimpy.Simulate()
time_libcosimpy = time.perf_counter() - t0

print("\nRunning on fmpy backend...")
t0 = time.perf_counter()
instance_fmpy = ShipInTransitCoSimulationFmpy(config=copy.deepcopy(config), spawn_requests=spawn_requests, ROOT=ROOT)
instance_fmpy.Simulate()
time_fmpy = time.perf_counter() - t0

# =========================
# Check both backends produced the same ship trajectories
# =========================
print("\nComparing outputs...")
outputs_match = True
for ship_id in ship_ids:
    for var in ("north", "east", "yaw_angle_rad", "forward_speed"):
        key                     = f"{ship_id}.{var}"
        _, _, libcosimpy_values = instance_libcosimpy.GetObserverTimeSeries(key)
        _, _, fmpy_values       = instance_fmpy.GetObserverTimeSeries(key)
        max_abs_diff            = np.max(np.abs(np.asarray(libcosimpy_values) - np.asarray(fmpy_values)))
        match                   = max_abs_diff < 1e-6
        outputs_match           = outputs_match and match
        print(f"  {key}: {'match' if match else 'MISMATCH'} (max abs diff = {max_abs_diff:.3g})")

print(f"\nOutputs match: {outputs_match}")

# =========================
# Compare time taken to solve the scenario
# =========================
print(f"\nlibcosimpy solve time: {time_libcosimpy:.3f} s")
print(f"fmpy solve time:       {time_fmpy:.3f} s")
print(f"fmpy speedup:          {time_libcosimpy / time_fmpy:.2f}x")
