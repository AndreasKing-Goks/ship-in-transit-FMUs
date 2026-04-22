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
    build_parameter_space_direct,
)
from orchestrator.scenario_config import (
    load_base_config,
    get_target_ship_ids,
    prepare_trial_config_direct,
)

# ─── Paths ───────────────────────────────────────────────────
CONFIG_PATH = Path(__file__).with_name("two_target_ship_ho.yaml")
SAVE_DIR = ROOT / "saved_animation"
BEST_ANIM_PATH = SAVE_DIR / "open_sea_two_ts_best_ax.mp4"
RESULTS_PATH = Path(__file__).with_name("ax_results.json")


# ─── Trial evaluation closure ────────────────────────────────
def evaluate_trial(parameters, _target_ship_ids=None):
    """Run one Ax trial: prepare config → simulate → compute metrics."""
    config = prepare_trial_config_direct(parameters, CONFIG_PATH, _target_ship_ids)
    instance = run_simulation(config, ROOT)
    metrics = compute_trial_metrics(instance, num_target_ships=len(_target_ship_ids))
    return {"objective": (metrics["objective"], 0.0)}, metrics


def main():
    """Run BO, persist results, and optionally replay the best trial."""
    num_sobol_trials = 2
    num_bo_trials = 2
    total_trials = num_sobol_trials + num_bo_trials
    replay_best = True

    base_config = load_base_config(CONFIG_PATH)
    target_ship_ids = get_target_ship_ids(base_config)

    print(
        f"Running Ax optimization with {num_sobol_trials} Sobol trials "
        f"and {num_bo_trials} BO trials (total={total_trials}), "
        f"target ships: {target_ship_ids}."
    )

    ax_client, best_parameters, history = run_ax_optimization(
        experiment_name="open_sea_two_ts_ho_bo",
        parameter_space=build_parameter_space_direct(target_ship_ids),
        evaluate_fn=lambda p: evaluate_trial(p, target_ship_ids),
        total_trials=total_trials,
        num_sobol_trials=num_sobol_trials,
        random_seed=7,
    )

    save_results(ax_client, best_parameters, history, RESULTS_PATH, trim=True)
    print_trial_summary_table(ax_client)

    if best_parameters is None:
        print("No valid Ax trials were completed. Check trial errors in history.")
        return

    print("Best parameters found:")
    for key, value in best_parameters.items():
        print(f"  {key}: {value}")

    if replay_best:
        replay_best_trial(
            config_preparer_fn=lambda p: prepare_trial_config_direct(p, CONFIG_PATH, target_ship_ids),
            target_ship_ids=target_ship_ids,
            best_parameters=best_parameters,
            root=ROOT,
            save_dir=SAVE_DIR,
            best_anim_path=BEST_ANIM_PATH,
        )


if __name__ == "__main__":
    main()