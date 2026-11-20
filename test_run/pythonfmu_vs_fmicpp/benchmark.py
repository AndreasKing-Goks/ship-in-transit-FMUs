"""
Compare the speed of the PythonFMU FMUs (FMUs/) and the C++ fmicpp FMUs (FMUs_cpp/).

Part 1: each FMU on its own (fmpy). Time to load and initialize, time per doStep and time to reset.
Part 2: a bank of traffic generator scenarios, on both backends, with three FMU setups:
        - all Python FMUs
        - C++ FMUs for the target ships only (FMUs/simplified_ship/)
        - C++ FMUs for every ported block (target ships and own ship)
        libcosimpy uses a new instance per case. fmpy reuses one instance with RespawnAndReset(),
        like in training. For fmpy, the time spent inside the FMUs' doStep is also measured, to
        show how much of the run time is FMU work and how much is the Python orchestrator.

Results are printed and saved to benchmark_results.csv next to this script.
Run parity.py first: the speed numbers only mean something if the results match.
"""

from pathlib import Path
import copy
import csv
import time

import numpy as np

# Shared helpers and settings from the parity check (same folder)
from parity import (ROOT, CONFIG_PATH, ENCOUNTER_SETTINGS_PATH, PORTED_FMUS, FMU_TESTS, STEP_SIZE,
                    Fmu, with_cpp_fmus,
                    ShipInTransitCoSimulationLibcosimpy, ShipInTransitCoSimulationFmpy,
                    load_base_config, generate_spawn_request_bank, load_spawn_requests_bank_path)

N_STEPS = 2000   # Part 1: steps per FMU
N_CASES = 5      # Part 2: scenarios per setup and backend

HERE = Path(__file__).resolve().parent
RESULTS_PATH = HERE / "benchmark_results.csv"


# =========================
# Part 1: each FMU on its own
# =========================
def time_single_fmu(path, params, inputs_fn):
    """Returns (load + initialize time [s], doStep times [s], reset + initialize time [s])."""
    t0 = time.perf_counter()
    fmu = Fmu(path)
    fmu.initialize(params)
    t_init = time.perf_counter() - t0

    rng = np.random.default_rng(1)
    step_times = np.empty(N_STEPS)
    t = 0.0
    for k in range(N_STEPS):
        fmu.set(inputs_fn(k, rng))
        t0 = time.perf_counter()
        fmu.fmu.doStep(currentCommunicationPoint=t, communicationStepSize=STEP_SIZE)
        step_times[k] = time.perf_counter() - t0
        t += STEP_SIZE

    t0 = time.perf_counter()
    fmu.fmu.reset()
    fmu.initialize(params)
    t_reset = time.perf_counter() - t0

    fmu.close()
    return t_init, step_times, t_reset


def benchmark_single_fmus(rows):
    print("=" * 106)
    print(f"Part 1: each FMU on its own (fmpy, {N_STEPS} steps)")
    print("=" * 106)
    print(f"{'FMU':32s} {'load+init [ms]':>16s} {'doStep median [us]':>20s} {'doStep p95 [us]':>17s} "
          f"{'reset [ms]':>12s} {'step speed-up':>14s}")
    print(f"{'':32s} {'py':>7s} {'cpp':>8s} {'py':>10s} {'cpp':>9s} {'py':>8s} {'cpp':>8s} {'py':>6s} {'cpp':>5s}")
    for name, fmu, params, inputs_fn in FMU_TESTS:
        py  = time_single_fmu(ROOT / "FMUs" / fmu, params, inputs_fn)
        cpp = time_single_fmu(ROOT / "FMUs_cpp" / fmu, params, inputs_fn)
        speed_up = np.median(py[1]) / np.median(cpp[1])
        print(f"{name:32s} {py[0]*1e3:7.0f} {cpp[0]*1e3:8.0f} "
              f"{np.median(py[1])*1e6:10.1f} {np.median(cpp[1])*1e6:9.1f} "
              f"{np.percentile(py[1], 95)*1e6:8.1f} {np.percentile(cpp[1], 95)*1e6:8.1f} "
              f"{py[2]*1e3:6.1f} {cpp[2]*1e3:5.1f} {speed_up:13.1f}x")
        for label, (t_init, step_times, t_reset) in (("python", py), ("cpp", cpp)):
            rows.append({"part": "single_fmu", "name": name, "fmus": label, "backend": "fmpy",
                         "load_init_s": t_init, "step_median_s": np.median(step_times),
                         "step_p95_s": np.percentile(step_times, 95), "reset_s": t_reset})


# =========================
# Part 2: scenarios
# =========================
def with_cpp_target_ships(config):
    """Copy of the config where only the target ships (FMUs/simplified_ship/) use the C++ FMUs."""
    config = copy.deepcopy(config)
    for ship in config["ships"]:
        for block in ship["SHIP_BLOCKS"]:
            if block[1].startswith("FMUs/simplified_ship/"):
                block[1] = PORTED_FMUS.get(block[1], block[1])
    return config


def count_steps(instance):
    """Number of steps that were simulated (length of any observed time series)."""
    key = next(iter(instance.observer_time_series_struct))
    return len(instance.GetObserverTimeSeries(key)[0])


def time_fmu_do_steps(instance):
    """Wrap doStep of every FMU in an fmpy instance, to add up the time spent inside the FMUs.
    Returns a one-element list with the total time [s]."""
    total = [0.0]
    for slave_data in instance.slaves.values():
        fmu = slave_data["fmu"]
        do_step = fmu.doStep

        def timed_do_step(*args, _do_step=do_step, **kwargs):
            t0 = time.perf_counter()
            result = _do_step(*args, **kwargs)
            total[0] += time.perf_counter() - t0
            return result

        fmu.doStep = timed_do_step
    return total


def run_libcosimpy(config, spawn_request_bank):
    """New instance per case. Returns (setup time per case [s], simulate time per case [s], steps)."""
    setup_times, sim_times, steps = [], [], 0
    for spawn_requests in spawn_request_bank:
        t0 = time.perf_counter()
        instance = ShipInTransitCoSimulationLibcosimpy(config=copy.deepcopy(config), spawn_requests=spawn_requests, ROOT=ROOT)
        setup_times.append(time.perf_counter() - t0)
        t0 = time.perf_counter()
        instance.Simulate()
        sim_times.append(time.perf_counter() - t0)
        steps += count_steps(instance)
    return setup_times, sim_times, steps, None


def run_fmpy(config, spawn_request_bank):
    """One instance, RespawnAndReset() between cases. The setup time of the first case is the
    constructor, the others are RespawnAndReset(). Also returns the time inside the FMUs' doStep."""
    setup_times, sim_times, steps = [], [], 0
    instance = None
    fmu_time = None
    for spawn_requests in spawn_request_bank:
        t0 = time.perf_counter()
        if instance is None:
            instance = ShipInTransitCoSimulationFmpy(config=copy.deepcopy(config), spawn_requests=spawn_requests, ROOT=ROOT)
            fmu_time = time_fmu_do_steps(instance)
        else:
            instance.RespawnAndReset(config=config, spawn_requests=spawn_requests, ROOT=ROOT)
        setup_times.append(time.perf_counter() - t0)
        t0 = time.perf_counter()
        instance.Simulate()
        sim_times.append(time.perf_counter() - t0)
        steps += count_steps(instance)
    return setup_times, sim_times, steps, fmu_time[0]


def benchmark_scenarios(rows):
    print()
    print("=" * 90)
    print(f"Part 2: {N_CASES} traffic generator scenarios per setup and backend")
    print("=" * 90)
    config = load_base_config(CONFIG_PATH)
    setups = [("all Python", config),
              ("C++ target ships", with_cpp_target_ships(config)),
              ("C++ all ported", with_cpp_fmus(config))]

    # New scenarios every run (the bank file is in .gitignore)
    bank_path = HERE / "spawn_request_bank.pkl"
    generate_spawn_request_bank(ROOT=ROOT,
                                config_path=CONFIG_PATH,
                                encounter_settings_path=ENCOUNTER_SETTINGS_PATH,
                                spawn_requests_bank_path=bank_path,
                                n_cases=N_CASES,
                                overwrite=True)
    bank = load_spawn_requests_bank_path(bank_path)
    spawn_request_bank = [case["spawn_requests"] for case in bank["cases"]]

    results = []
    for backend, run in (("libcosimpy", run_libcosimpy), ("fmpy", run_fmpy)):
        for setup_name, setup_config in setups:
            print(f"  running {backend:10s} {setup_name} ...", flush=True)
            setup_times, sim_times, steps, fmu_time = run(setup_config, spawn_request_bank)
            results.append((backend, setup_name, setup_times, sim_times, steps, fmu_time))

    print()
    print(f"{'backend':10s} {'FMUs':18s} {'setup [s]':>10s} {'simulate [s]':>13s} {'per step [ms]':>14s} "
          f"{'speed-up':>9s} {'in FMU doStep':>14s}")
    for backend, setup_name, setup_times, sim_times, steps, fmu_time in results:
        baseline = next(r for r in results if r[0] == backend and r[1] == "all Python")
        speed_up = sum(baseline[3]) / sum(sim_times)
        fmu_share = f"{100 * fmu_time / sum(sim_times):13.0f}%" if fmu_time is not None else f"{'-':>14s}"
        print(f"{backend:10s} {setup_name:18s} {np.mean(setup_times):10.2f} {np.mean(sim_times):13.2f} "
              f"{1e3 * sum(sim_times) / steps:14.2f} {speed_up:8.2f}x {fmu_share}")
        rows.append({"part": "scenario", "name": setup_name, "fmus": setup_name, "backend": backend,
                     "load_init_s": np.mean(setup_times), "simulate_s": np.mean(sim_times),
                     "step_mean_s": sum(sim_times) / steps,
                     "fmu_do_step_share": fmu_time / sum(sim_times) if fmu_time is not None else ""})
    print()
    print("setup = constructor (libcosimpy: every case; fmpy: first case) or RespawnAndReset() (fmpy: other cases),"
          " averaged over the cases")


if __name__ == "__main__":
    rows = []
    benchmark_single_fmus(rows)
    benchmark_scenarios(rows)

    fields = ["part", "name", "fmus", "backend", "load_init_s", "step_median_s", "step_p95_s", "reset_s",
              "simulate_s", "step_mean_s", "fmu_do_step_share"]
    with RESULTS_PATH.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    print(f"\nSaved {RESULTS_PATH.relative_to(ROOT)}")
