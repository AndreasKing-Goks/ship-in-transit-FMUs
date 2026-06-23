"""
Minimal example: build a bank of ~10 different trafficgen-generated scenarios directly
from the trafficgen config file, then run all of them on both Ship-in-Transit
co-simulation backends - libcosimpy (orchestrator/sit_cosim.py) and fmpy
(orchestrator/sit_cosim_fmpy.py) - and check that every case produces the same ship
trajectories on both backends, and compare the total time taken to solve the whole bank.

Unlike main.py (one scenario, one fresh instance per backend), this also exercises
fmpy's Reset(): the libcosimpy side creates a brand new instance for every case (it has
no working Reset()), while the fmpy side creates ONE instance up front and reuses it for
every case via Reset() + re-staging that case's spawn positions/routes - the realistic
pattern for sweeping many scenarios (e.g. Monte Carlo) without paying FMU
re-instantiation cost on every case.

Reuses the trafficgen example config from test_run/traffic_generator_test/, and the
existing bank-of-scenarios machinery in orchestrator/scenario_config.py
(generate_spawn_request_bank / load_spawn_requests_bank_path).
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
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from orchestrator.sit_cosim import ShipInTransitCoSimulation as ShipInTransitCoSimulationLibcosimpy
from orchestrator.sit_cosim_fmpy import ShipInTransitCoSimulation as ShipInTransitCoSimulationFmpy
from orchestrator.scenario_config import (load_base_config,
                                          generate_spawn_request_bank,
                                          load_spawn_requests_bank_path)
from orchestrator.utils import compile_ship_params

N_CASES = 10

config_path              = ROOT / "test_run" / "traffic_generator_test" / "traffic_gen.yaml"
encounter_settings_path  = ROOT / "test_run" / "traffic_generator_test" / "encounter_settings.json"
spawn_requests_bank_path = HERE / "spawn_request_bank.pkl"


def get_trajectory(instance, ship_ids):
    traj = {}
    for ship_id in ship_ids:
        for var in ("north", "east", "yaw_angle_rad", "forward_speed"):
            key = f"{ship_id}.{var}"
            _, _, values = instance.GetObserverTimeSeries(key)
            traj[key] = np.asarray(values, dtype=float)
    return traj


def respawn_fmpy_instance(instance, config, spawn_requests):
    """
        Stage a new case's spawn positions/routes on an existing fmpy
        ShipInTransitCoSimulation instance, ready for the next Reset() call to apply.

        Must be called before Reset(): the ship's initial position/yaw/speed are FMI2
        "fixed"-variability parameters, which can only legally change while the FMU is
        in initialization mode. SetInitialValues() only stages self.initial_values;
        it is Reset()'s own _initialize() that re-enters initialization mode and applies
        them properly, then rebuilds the SIT bookkeeping (stop_info,
        ship_reach_end_waypoint, ...) using the ship_configs set here.
    """
    ship_configs = instance.AddShipSpawn(spawn_requests=spawn_requests,
                                         ROOT=ROOT,
                                         simu_config=config["simulation"],
                                         ship_configs=copy.deepcopy(config["ships"]))
    instance.ship_configs = ship_configs
    instance.ship_idxs    = [ship_config["id"] for ship_config in ship_configs]

    for ship_config in ship_configs:
        prefix     = ship_config["id"]
        fmu_params = compile_ship_params(ship_config)
        fmu_params = instance.spawn_ship(ship_id=prefix, spawn=ship_config["spawn"], fmu_params=fmu_params)
        for block, params in fmu_params.items():
            instance.SetInitialValues(slaveName=instance.ship_slave(prefix, block), params=params)


# =========================
# Build a bank of N_CASES trafficgen scenarios straight from the config file
# =========================
print(f"Generating a bank of {N_CASES} trafficgen scenarios...")
generate_spawn_request_bank(
    ROOT=ROOT,
    config_path=config_path,
    encounter_settings_path=encounter_settings_path,
    spawn_requests_bank_path=spawn_requests_bank_path,
    n_cases=N_CASES,
    overwrite=True,
)
bank                = load_spawn_requests_bank_path(spawn_requests_bank_path)
spawn_request_bank  = [case["spawn_requests"] for case in bank["cases"]]

config   = load_base_config(config_path)
ship_ids = [ship_config["id"] for ship_config in config["ships"]]

# =========================
# libcosimpy: a fresh instance per case (no working Reset() on this backend)
# =========================
print(f"\n=== Running {N_CASES} cases on libcosimpy (fresh instance per case) ===")
libcosimpy_trajectories = []
t0 = time.perf_counter()
for case_idx, spawn_requests in enumerate(spawn_request_bank):
    instance = ShipInTransitCoSimulationLibcosimpy(config=copy.deepcopy(config), spawn_requests=spawn_requests, ROOT=ROOT)
    instance.Simulate()
    libcosimpy_trajectories.append(get_trajectory(instance, ship_ids))
time_libcosimpy = time.perf_counter() - t0

# =========================
# fmpy: ONE instance, reused across all cases via Reset()
# =========================
print(f"\n=== Running {N_CASES} cases on fmpy (one instance, Reset() between cases) ===")
fmpy_trajectories = []
fmpy_instance      = None
t0 = time.perf_counter()
for case_idx, spawn_requests in enumerate(spawn_request_bank):
    if fmpy_instance is None:
        fmpy_instance = ShipInTransitCoSimulationFmpy(config=copy.deepcopy(config), spawn_requests=spawn_requests, ROOT=ROOT)
    else:
        respawn_fmpy_instance(fmpy_instance, config, spawn_requests)
        fmpy_instance.Reset()
    fmpy_instance.Simulate()
    fmpy_trajectories.append(get_trajectory(fmpy_instance, ship_ids))
time_fmpy = time.perf_counter() - t0

# =========================
# Compare outputs case by case
# =========================
print("\nComparing outputs per case...")
all_match = True
for case_idx in range(N_CASES):
    case_match   = True
    max_abs_diff = 0.0
    for ship_id in ship_ids:
        for var in ("north", "east", "yaw_angle_rad", "forward_speed"):
            key  = f"{ship_id}.{var}"
            lc   = libcosimpy_trajectories[case_idx][key]
            fm   = fmpy_trajectories[case_idx][key]
            n    = min(len(lc), len(fm))
            diff = float(np.max(np.abs(lc[:n] - fm[:n]))) if n > 0 else float("nan")
            max_abs_diff = max(max_abs_diff, diff)
            case_match   = case_match and n > 0 and diff < 1e-6
    all_match = all_match and case_match
    print(f"  case {case_idx}: {'match' if case_match else 'MISMATCH'} (max abs diff = {max_abs_diff:.3g})")

print(f"\nAll {N_CASES} cases match: {all_match}")

# =========================
# Compare total time taken to solve the bank
# =========================
print(f"\nlibcosimpy total solve time ({N_CASES} cases): {time_libcosimpy:.3f} s")
print(f"fmpy total solve time       ({N_CASES} cases): {time_fmpy:.3f} s")
print(f"fmpy speedup:                                  {time_libcosimpy / time_fmpy:.2f}x")
