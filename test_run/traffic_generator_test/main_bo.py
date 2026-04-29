""" 
Bayesian Optimization for one target ship in open sea scenario, using traffic generator for config preparation. 
This is a more complex example than the direct parameterization version, as it involves generating intermediate waypoints and converting them to FMU parameters.
Ship traffic generator implementation is used to create realistic encounter scenarios based on high-level parameters, which are then converted to simulation configs for each trial.
The BO workflow remains the same, but the parameter space and config preparation are more involved due to the use of traffic generator. 
Results are saved and the best trial can be replayed with animation.

Author:
Melih Akdağ
Date: 2026.04.22
"""

from pathlib import Path
import sys
import os

# Ensure libcosim DLL is found
dll_dir = Path(sys.prefix) / "Lib" / "site-packages" / "libcosimpy" / "libcosimc"
os.add_dll_directory(str(dll_dir))

# Project root = two levels up from this file
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from orchestrator.bo_core import (
    run_simulation,
    compute_trial_metrics,
    run_ax_optimization,
    replay_best_trial,
    save_results,
    print_trial_summary_table,
    build_parameter_space_trafficgen,
)
from orchestrator.scenario_config import (
    prepare_trial_config_trafficgen,
)


# ─── Paths ───────────────────────────────────────────────────
CONFIG_PATH = Path(__file__).with_name("single_target_ship_ho.yaml")
SAVE_DIR = ROOT / "saved_animation"
BEST_ANIM_PATH = SAVE_DIR / "open_sea_one_ts_best_ax.mp4"
RESULTS_PATH = Path(__file__).with_name("ax_results.json")
OWN_SHIP_PATH = ROOT / "data" / "ownship" / "ownship.json"
TARGET_SHIPS_PATH = ROOT / "data" / "targetships"
ENCOUNTER_SETTINGS_PATH = Path(__file__).with_name("encounter_settings.json")


# ─── Fixed scenario decisions ────────────────────────────────
ENCOUNTERS = [
    {"id": "TS1", "encounter_type": "head-on"},
    # {"id": "TS2", "encounter_type": "crossing-give-way"},
]

OS_INITIAL = {
    "position": {"lat": 58.763449, "lon": 10.490654},
    "sog": 10.0,
    "cog": 0.0,
    "heading": 0.0,
    "navStatus": "Under way using engine",
}

TARGET_SHIP_IDS = [enc["id"] for enc in ENCOUNTERS]


# ─── Trial evaluation closure ────────────────────────────────
def _prepare_config(parameters):
    """Build a ready-to-run config from BO parameters via trafficgen."""
    return prepare_trial_config_trafficgen(
        parameters, CONFIG_PATH, ENCOUNTERS, OS_INITIAL,
        OWN_SHIP_PATH, TARGET_SHIPS_PATH, ENCOUNTER_SETTINGS_PATH,
    )


def evaluate_trial(parameters):
    """Run one Ax trial: generate → convert → simulate → metrics."""
    config = _prepare_config(parameters)
    instance = run_simulation(config, ROOT)
    metrics = compute_trial_metrics(instance, num_target_ships=len(ENCOUNTERS))
    return {"objective": (metrics["objective"], 0.0)}, metrics


def main():
    """Run BO, persist results, and optionally replay the best trial."""
    num_sobol_trials = 2
    num_bo_trials = 2
    total_trials = num_sobol_trials + num_bo_trials
    replay_best = True
    max_intermediate_wps = 1

    print(
        f"Running Ax optimization with {num_sobol_trials} Sobol trials "
        f"and {num_bo_trials} BO trials (total={total_trials}), "
        f"max intermediate waypoints: {max_intermediate_wps}."
    )

    ax_client, best_parameters, history = run_ax_optimization(
        experiment_name="open_sea_one_ts_ho_bo_stg",
        parameter_space=build_parameter_space_trafficgen(
            ENCOUNTERS, ENCOUNTER_SETTINGS_PATH, max_intermediate_wps=max_intermediate_wps,
        ),
        evaluate_fn=evaluate_trial,
        total_trials=total_trials,
        num_sobol_trials=num_sobol_trials,
        random_seed=7,
    )

    save_results(ax_client, best_parameters, history, RESULTS_PATH, trim=False)
    print_trial_summary_table(ax_client)

    if best_parameters is None:
        print("No valid Ax trials were completed. Check trial errors in history.")
        return

    print("Best parameters found:")
    for key, value in best_parameters.items():
        print(f"  {key}: {value}")

    if replay_best:
        replay_best_trial(
            config_preparer_fn=_prepare_config,
            target_ship_ids=TARGET_SHIP_IDS,
            best_parameters=best_parameters,
            root=ROOT,
            save_dir=SAVE_DIR,
            best_anim_path=BEST_ANIM_PATH,
        )


if __name__ == "__main__":
    main()