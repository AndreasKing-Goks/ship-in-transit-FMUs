"""
Check that the C++ (fmicpp) FMUs in FMUs_cpp/ give the same results as the PythonFMU FMUs in FMUs/.

Part 1: each FMU on its own (fmpy). Checks that the C++ FMU has the same variables as the Python
        FMU, and gives the same outputs for the same parameters and random inputs. Random inputs
        also reach cases that a normal scenario never does (e.g. rudder saturation).
Part 2: the existing simulation loops, as in test_run/fmpy_vs_libcosimpy_test/main_loop.py.
        A bank of traffic generator scenarios is run with the Python FMUs and with the C++ FMUs.
        libcosimpy uses a new instance per case. fmpy reuses one instance with RespawnAndReset(),
        like in training, so this also checks reset.
"""

from pathlib import Path
import sys
import os
import copy
import shutil
import tempfile

import numpy as np
from fmpy import extract, read_model_description
from fmpy.fmi2 import FMU2Slave

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
                                          generate_spawn_request_bank,
                                          load_spawn_requests_bank_path)

# Part 1: results must match within |cpp - py| <= ATOL + RTOL * |py|
ATOL = 1e-9
RTOL = 1e-9
STEP_SIZE = 5.0
N_STEPS   = 2000

# Part 2: closed loop over many steps, so allow a bit more
SCENARIO_TOL = 1e-6
N_CASES      = 10

CONFIG_PATH             = ROOT / "test_run" / "traffic_generator_test" / "traffic_gen.yaml"
ENCOUNTER_SETTINGS_PATH = ROOT / "test_run" / "traffic_generator_test" / "encounter_settings.json"

# Python FMU -> C++ FMU
PORTED_FMUS = {
    "FMUs/simplified_ship/MissionManager.fmu":  "FMUs_cpp/simplified_ship/MissionManager.fmu",
    "FMUs/simplified_ship/Autopilot.fmu":       "FMUs_cpp/simplified_ship/Autopilot.fmu",
    "FMUs/simplified_ship/Rudder.fmu":          "FMUs_cpp/simplified_ship/Rudder.fmu",
    "FMUs/simplified_ship/ShipModel.fmu":       "FMUs_cpp/simplified_ship/ShipModel.fmu",
    "FMUs/simplified_ship/SpeedController.fmu": "FMUs_cpp/simplified_ship/SpeedController.fmu",
    # The own ship uses the same Python scripts, built into FMUs/ship/
    "FMUs/ship/MissionManager.fmu":             "FMUs_cpp/simplified_ship/MissionManager.fmu",
    "FMUs/ship/Autopilot.fmu":                  "FMUs_cpp/simplified_ship/Autopilot.fmu",
    "FMUs/ship/Rudder.fmu":                     "FMUs_cpp/simplified_ship/Rudder.fmu",
    "FMUs/ship/ShipModel.fmu":                  "FMUs_cpp/simplified_ship/ShipModel.fmu",
}


# =========================
# Part 1: each FMU on its own
# =========================
def ts1_params(block):
    """Parameters of a target ship block in traffic_gen.yaml."""
    config = load_base_config(CONFIG_PATH)
    ts1 = next(s for s in config["ships"] if s["id"] == "TS1")
    return dict(ts1["fmu_params"][block])


# Waypoints for the mission manager and a path that follows them
ROUTE = [(0.0, 0.0, 5.0), (2000.0, 500.0, 6.0), (3500.0, 2500.0, 4.0), (6000.0, 3000.0, 5.0)]


def route_position(k):
    """Ship position after k steps along ROUTE, at 20 m per step."""
    pts = np.array([(n, e) for n, e, _ in ROUTE])
    s = 20.0 * k
    for a, b in zip(pts[:-1], pts[1:]):
        seg = np.linalg.norm(b - a)
        if s <= seg:
            return a + (b - a) * s / seg
        s -= seg
    return pts[-1]


def mission_manager_params():
    params = {"ra": 300.0, "max_inner_wp": 2}
    params["wp_start_north"], params["wp_start_east"], params["wp_start_speed"] = ROUTE[0]
    params["wp_1_north"], params["wp_1_east"], params["wp_1_speed"] = ROUTE[1]
    params["wp_2_north"], params["wp_2_east"], params["wp_2_speed"] = ROUTE[2]
    params["wp_end_north"], params["wp_end_east"], params["wp_end_speed"] = ROUTE[3]
    return params


def mission_manager_inputs(k, rng):
    n, e = route_position(k)
    return {"north": n + rng.normal(0, 5), "east": e + rng.normal(0, 5)}


def autopilot_inputs(k, rng):
    return {"north": rng.uniform(-500, 3000), "east": rng.uniform(-500, 3000),
            "yaw_angle_rad": rng.uniform(-np.pi, np.pi),
            "prev_wp_north": 0.0, "prev_wp_east": 0.0,
            "next_wp_north": 2000.0, "next_wp_east": 500.0}


def rudder_inputs(k, rng):
    return {"rudder_angle_deg": rng.uniform(-40, 40), "yaw_angle_rad": rng.uniform(-np.pi, np.pi),
            "forward_speed": rng.uniform(0, 8), "current_speed": rng.uniform(0, 1),
            "current_dir_rad": rng.uniform(-np.pi, np.pi)}


def ship_model_inputs(k, rng):
    return {"thrust_force": rng.uniform(0, 4e5), "rudder_force_v": rng.uniform(-1e5, 1e5),
            "rudder_force_r": rng.uniform(-1e6, 1e6), "wind_speed": rng.uniform(0, 15),
            "wind_dir_rad": rng.uniform(-np.pi, np.pi), "current_speed": rng.uniform(0, 1),
            "current_dir_rad": rng.uniform(-np.pi, np.pi)}


def speed_controller_inputs(k, rng):
    return {"desired_ship_speed": rng.uniform(0, 8), "measured_ship_speed": rng.uniform(0, 8)}


ship_model_params = ts1_params("SHIP_MODEL")
ship_model_params.update({"initial_north_position_m": 100.0, "initial_east_position_m": -50.0,
                          "initial_yaw_angle_rad": 0.5, "initial_forward_speed_m_per_s": 4.0})

FMU_TESTS = [
    # name, parameters, input function
    ("MissionManager",  mission_manager_params(),       mission_manager_inputs),
    ("Autopilot",       ts1_params("AUTOPILOT"),        autopilot_inputs),
    ("Rudder",          ts1_params("RUDDER"),           rudder_inputs),
    ("ShipModel",       ship_model_params,              ship_model_inputs),
    ("SpeedController", ts1_params("SPEED_CONTROLLER"), speed_controller_inputs),
]


class Fmu:
    """Small fmpy wrapper that sets and gets variables by name."""

    def __init__(self, path):
        self.md = read_model_description(str(path))
        self.vars = {v.name: v for v in self.md.modelVariables}
        self.unzip_dir = extract(str(path), unzipdir=tempfile.mkdtemp())
        self.fmu = FMU2Slave(guid=self.md.guid, unzipDirectory=self.unzip_dir,
                             modelIdentifier=self.md.coSimulation.modelIdentifier, instanceName="parity")
        self.fmu.instantiate()

    def set(self, values):
        for name, value in values.items():
            v = self.vars[name]
            if v.type == "Real":
                self.fmu.setReal([v.valueReference], [float(value)])
            elif v.type == "Integer":
                self.fmu.setInteger([v.valueReference], [int(value)])
            elif v.type == "Boolean":
                self.fmu.setBoolean([v.valueReference], [bool(value)])

    def get(self, name):
        v = self.vars[name]
        if v.type == "Real":
            return self.fmu.getReal([v.valueReference])[0]
        if v.type == "Integer":
            return self.fmu.getInteger([v.valueReference])[0]
        return self.fmu.getBoolean([v.valueReference])[0]

    def initialize(self, params):
        self.set(params)
        self.fmu.setupExperiment(startTime=0.0)
        self.fmu.enterInitializationMode()
        self.fmu.exitInitializationMode()

    def close(self):
        self.fmu.terminate()
        self.fmu.freeInstance()
        shutil.rmtree(self.unzip_dir, ignore_errors=True)


def check_interface(py, cpp):
    """The C++ FMU must have every Python variable, with the same type, causality and start value.
    Returns (problems, notes). Notes are known differences that are not errors."""
    problems = []
    notes = []
    for name, pv in py.vars.items():
        cv = cpp.vars.get(name)
        if cv is None:
            problems.append(f"{name}: missing in C++ FMU")
            continue
        for attr in ("type", "causality"):
            if getattr(pv, attr) != getattr(cv, attr):
                problems.append(f"{name}: {attr} {getattr(pv, attr)} (py) != {getattr(cv, attr)} (cpp)")
        if pv.causality == "parameter" and pv.variability != cv.variability:
            if pv.variability == "tunable" and cv.variability == "fixed":
                # fmicpp 0.5 writes tunable parameters as "fixed" in modelDescription.xml.
                # The C++ FMU still lets them be set after initialization.
                notes.append(f"{name}: tunable, but modelDescription.xml says fixed (fmicpp 0.5 bug)")
            else:
                problems.append(f"{name}: variability {pv.variability} (py) != {cv.variability} (cpp)")
        if pv.start is not None and cv.start is not None and float(pv.start) != float(cv.start):
            problems.append(f"{name}: start {pv.start} (py) != {cv.start} (cpp)")
    return problems, notes


def run_and_compare(py, cpp, inputs_fn, outputs):
    """Step both FMUs with the same inputs. Returns the number of output values out of
    tolerance, and the largest difference per output."""
    rng = np.random.default_rng(1)
    worst = {name: 0.0 for name in outputs}
    n_bad = 0
    t = 0.0
    for k in range(N_STEPS):
        inputs = inputs_fn(k, rng)
        py.set(inputs)
        cpp.set(inputs)
        py.fmu.doStep(currentCommunicationPoint=t, communicationStepSize=STEP_SIZE)
        cpp.fmu.doStep(currentCommunicationPoint=t, communicationStepSize=STEP_SIZE)
        t += STEP_SIZE
        for name in outputs:
            a, b = float(py.get(name)), float(cpp.get(name))
            if np.isnan(a) and np.isnan(b):
                continue
            diff = abs(a - b) if np.isfinite(a - b) else np.inf
            worst[name] = max(worst[name], diff)
            if diff > ATOL + RTOL * abs(a):
                n_bad += 1
    return n_bad, worst


def compare_single_fmus():
    print("=" * 70)
    print("Part 1: each FMU on its own")
    print("=" * 70)
    all_ok = True
    for name, params, inputs_fn in FMU_TESTS:
        py  = Fmu(ROOT / "FMUs" / "simplified_ship" / f"{name}.fmu")
        cpp = Fmu(ROOT / "FMUs_cpp" / "simplified_ship" / f"{name}.fmu")

        problems, notes = check_interface(py, cpp)
        outputs = [n for n, v in py.vars.items() if v.causality == "output"]

        # Outputs after initialization (before the first step)
        py.initialize(params)
        cpp.initialize(params)
        n_bad_init = sum(py.get(n) != cpp.get(n) for n in outputs)

        n_bad, worst = run_and_compare(py, cpp, inputs_fn, outputs)

        ok = not problems and n_bad_init == 0 and n_bad == 0
        all_ok = all_ok and ok
        print(f"\n{name}: {'OK' if ok else 'FAILED'}")
        for p in problems:
            print(f"  interface: {p}")
        for n in notes:
            print(f"  note: {n}")
        if n_bad_init:
            print(f"  {n_bad_init} outputs differ after initialization")
        print(f"  {n_bad} output values out of tolerance in {N_STEPS} steps")
        for n in outputs:
            print(f"    {n:20s} max abs diff = {worst[n]:.3g}")

        py.close()
        cpp.close()
    return all_ok


# =========================
# Part 2: existing simulation loops
# =========================
def with_cpp_fmus(config):
    """Copy of the config with the ported blocks pointing to the C++ FMUs."""
    config = copy.deepcopy(config)
    for ship in config["ships"]:
        for block in ship["SHIP_BLOCKS"]:
            block[1] = PORTED_FMUS.get(block[1], block[1])
    return config


def get_observed(instance):
    """All observed time series of an instance, by key ("<ship id>.<variable>")."""
    return {key: np.asarray(instance.GetObserverTimeSeries(key)[2], dtype=float)
            for key in instance.observer_time_series_struct}


def compare_case(py_series, cpp_series):
    """Returns (largest difference, keys that do not match)."""
    max_abs_diff = 0.0
    bad_keys = []
    for key, py_values in py_series.items():
        cpp_values = cpp_series[key]
        if len(py_values) != len(cpp_values) or len(py_values) == 0:
            bad_keys.append(key)
            continue
        # NaN in both (e.g. TCPA when the ships do not close in) counts as equal
        both_nan = np.isnan(py_values) & np.isnan(cpp_values)
        diff = np.abs(py_values - cpp_values)[~both_nan]
        if diff.size:
            max_abs_diff = max(max_abs_diff, float(np.max(diff)))
        if not np.allclose(cpp_values, py_values, atol=SCENARIO_TOL, rtol=SCENARIO_TOL, equal_nan=True):
            bad_keys.append(key)
    return max_abs_diff, bad_keys


def compare_scenarios():
    print()
    print("=" * 70)
    print(f"Part 2: {N_CASES} traffic generator scenarios")
    print("=" * 70)
    config     = load_base_config(CONFIG_PATH)
    config_cpp = with_cpp_fmus(config)

    # New scenarios every run (the bank file is in .gitignore)
    bank_path = Path(__file__).resolve().parent / "spawn_request_bank.pkl"
    generate_spawn_request_bank(ROOT=ROOT,
                                config_path=CONFIG_PATH,
                                encounter_settings_path=ENCOUNTER_SETTINGS_PATH,
                                spawn_requests_bank_path=bank_path,
                                n_cases=N_CASES,
                                overwrite=True)
    bank = load_spawn_requests_bank_path(bank_path)
    spawn_request_bank = [case["spawn_requests"] for case in bank["cases"]]

    all_ok = True

    # libcosimpy: a new instance per case (no working Reset() on this backend)
    print("\nlibcosimpy (new instance per case):")
    for case_idx, spawn_requests in enumerate(spawn_request_bank):
        py = ShipInTransitCoSimulationLibcosimpy(config=copy.deepcopy(config), spawn_requests=spawn_requests, ROOT=ROOT)
        py.Simulate()
        cpp = ShipInTransitCoSimulationLibcosimpy(config=copy.deepcopy(config_cpp), spawn_requests=spawn_requests, ROOT=ROOT)
        cpp.Simulate()
        max_abs_diff, bad_keys = compare_case(get_observed(py), get_observed(cpp))
        all_ok = all_ok and not bad_keys
        print(f"  case {case_idx}: {'match' if not bad_keys else 'MISMATCH'} (max abs diff = {max_abs_diff:.3g})")
        for key in bad_keys:
            print(f"    mismatch: {key}")

    # fmpy: one instance per FMU set, reused for every case with RespawnAndReset()
    print("\nfmpy (one instance, RespawnAndReset() between cases):")
    py = cpp = None
    for case_idx, spawn_requests in enumerate(spawn_request_bank):
        if py is None:
            py  = ShipInTransitCoSimulationFmpy(config=copy.deepcopy(config), spawn_requests=spawn_requests, ROOT=ROOT)
            cpp = ShipInTransitCoSimulationFmpy(config=copy.deepcopy(config_cpp), spawn_requests=spawn_requests, ROOT=ROOT)
        else:
            py.RespawnAndReset(config=config, spawn_requests=spawn_requests, ROOT=ROOT)
            cpp.RespawnAndReset(config=config_cpp, spawn_requests=spawn_requests, ROOT=ROOT)
        py.Simulate()
        cpp.Simulate()
        max_abs_diff, bad_keys = compare_case(get_observed(py), get_observed(cpp))
        all_ok = all_ok and not bad_keys
        print(f"  case {case_idx}: {'match' if not bad_keys else 'MISMATCH'} (max abs diff = {max_abs_diff:.3g})")
        for key in bad_keys:
            print(f"    mismatch: {key}")

    return all_ok


if __name__ == "__main__":
    single_ok = compare_single_fmus()
    scenarios_ok = compare_scenarios()
    print()
    print(f"Part 1 (each FMU on its own):   {'OK' if single_ok else 'FAILED'}")
    print(f"Part 2 (existing loops, reset): {'OK' if scenarios_ok else 'FAILED'}")
    sys.exit(0 if single_ok and scenarios_ok else 1)
