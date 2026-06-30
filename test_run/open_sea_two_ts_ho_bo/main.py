""" 
Bayesian Optimization for two target ships in open sea scenario, using direct parameter space construction and trial config preparation. 
This is a simplified version of the two target ship case, focused on validating the BO workflow and infrastructure with two target ships. 
The parameter space is built directly from the scenario config, and the trial evaluation runs a full simulation and metric computation for each set of parameters. 
Results are saved and the best trial can be replayed with animation.

Author:
Melih Akdağ
Date: 2026.04.22
"""

from pathlib import Path
import sys
import os
import copy

# Ensure libcosim DLL is found
dll_dir = Path(sys.prefix) / "Lib" / "site-packages" / "libcosimpy" / "libcosimc"
os.add_dll_directory(str(dll_dir))

# Project root = two levels up from this file
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from orchestrator.bo_core import (
    build_parameter_space_trafficgen,
    run_simulation,
    compute_trial_metrics,
    run_ax_optimization,
    replay_best_trial,
    save_results,
    print_trial_summary_table,
    prepare_spawn_requests_trafficgen,
)
from orchestrator.scenario_config import load_base_config
from orchestrator.utils import view_bo_results_table

# ─── Paths ───────────────────────────────────────────────────
CONFIG_PATH = Path(__file__).with_name("two_target_ship_ho.yaml")
SAVE_DIR = ROOT / "saved_animation"
BEST_ANIM_PATH = SAVE_DIR / "open_sea_two_ts_best_ax.mp4"
RESULTS_PATH = Path(__file__).with_name("ax_results.json")
ENCOUNTER_SETTINGS_PATH = Path(__file__).with_name("encounter_settings.json")


# ─── Fixed scenario decisions ────────────────────────────────
ENCOUNTERS = [
    {"id": "TS1", "encounter_type": "head-on"},
    {"id": "TS2", "encounter_type": "crossing-give-way"},
]

OS_INITIAL = {
    "position": {"north": 0.0, "east": 0.0},
    "sog": 10.0,
    "cog": 0.0,
    "heading": 0.0,
    "navStatus": "Under way using engine",
}

TARGET_SHIP_IDS = [enc["id"] for enc in ENCOUNTERS]



# ─── Trial evaluation closure ────────────────────────────────
def _prepare_scenario(parameters):
    """Build (base config, trafficgen spawn requests) from BO parameters."""
    config = copy.deepcopy(load_base_config(CONFIG_PATH))
    spawn_requests = prepare_spawn_requests_trafficgen(
        parameters, ENCOUNTERS, OS_INITIAL, CONFIG_PATH, ENCOUNTER_SETTINGS_PATH,
    )
    return config, spawn_requests

def evaluate_trial(parameters):
    """Run one Ax trial: prepare scenario → simulate → compute metrics."""
    config, spawn_requests = _prepare_scenario(parameters)
    instance = run_simulation(config, ROOT, spawn_requests=spawn_requests)
    metrics = compute_trial_metrics(instance, num_target_ships=len(ENCOUNTERS))
    return {"objective": (metrics["objective"], 0.0)}, metrics


def main():
    """Run BO, persist results, and optionally replay the best trial."""
    num_sobol_trials = 50
    num_bo_trials = 150
    total_trials = num_sobol_trials + num_bo_trials
    replay_best = False

    print(
        f"Running Ax optimization with {num_sobol_trials} Sobol trials "
        f"and {num_bo_trials} BO trials (total={total_trials}), "
        f"optimizing trafficgen initial conditions for {TARGET_SHIP_IDS}."
    )

    ax_client, best_parameters, history = run_ax_optimization(
        experiment_name="open_sea_two_ts_ho_bo",
        parameter_space=build_parameter_space_trafficgen(ENCOUNTERS, ENCOUNTER_SETTINGS_PATH),
        evaluate_fn=evaluate_trial,
        total_trials=total_trials,
        num_sobol_trials=num_sobol_trials,
        random_seed=7,
    )

    save_results(ax_client, best_parameters, history, RESULTS_PATH, trim=False)
    print_trial_summary_table(ax_client)
    
    # Always create CSV results
    view_bo_results_table(RESULTS_PATH, save_csv=True)

    if best_parameters is None:
        print("No valid Ax trials were completed. Check trial errors in history.")
        return

    print("Best parameters found:")
    for key, value in best_parameters.items():
        print(f"  {key}: {value}")

    if replay_best:
        replay_best_trial(
            scenario_preparer_fn=_prepare_scenario,
            target_ship_ids=TARGET_SHIP_IDS,
            best_parameters=best_parameters,
            root=ROOT,
            save_dir=SAVE_DIR,
            best_anim_path=BEST_ANIM_PATH,
        )


if __name__ == "__main__":
    main()