"""Bayesian Optimization core: simulation helpers, metrics, Ax loop, replay, and results."""

import copy
import json
import re
import traceback

import numpy as np
import pandas as pd
from ax.service.ax_client import AxClient, ObjectiveProperties

from orchestrator.sit_cosim import ShipInTransitCoSimulation


# ═════════════════════════════════════════════════════════════
# Simulation helpers
# ═════════════════════════════════════════════════════════════
def build_instance(config, root):
    """Create one co-simulation instance from a prepared config."""
    return ShipInTransitCoSimulation(config=config, ROOT=root)


def run_simulation(config, root):
    """Build and run one simulation trial, then return the instance."""
    instance = build_instance(config, root)
    instance.Simulate()
    return instance


# ═════════════════════════════════════════════════════════════
# Safety metrics
# ═════════════════════════════════════════════════════════════
def _capped_margin(value, threshold):
    """Return a normalized safety margin in [0, 1], where 0 is worst."""
    if not np.isfinite(value):
        return 1.0
    if threshold <= 0:
        return 1.0
    return float(np.clip(value / threshold, 0.0, 1.0))


def compute_trial_metrics(instance, num_target_ships):
    """Extract safety-based trial metrics and combine them into one BO objective.

    Generalised to N target ships: loops over each target, takes worst-case
    margins, and records per-target detail keys.
    """
    # Path-following performance
    _, _, e_ct = instance.get_ship_timeseries("OS0", "e_ct")
    e_ct = np.asarray(e_ct, dtype=float)

    reached_end = bool(
        instance.GetLastValue(
            slaveName=instance.ship_slave("OS0", "MISSION_MANAGER"),
            slaveVar="reach_wp_end",
        )
    )

    collision = bool(
        instance.GetLastValue(
            slaveName=instance.ship_slave("OS0", "COLAV"),
            slaveVar="ship_collision",
        )
    )

    fuel_consumption = float(
        instance.GetLastValue(
            slaveName=instance.ship_slave("OS0", "MACHINERY_SYSTEM"),
            slaveVar="fuel_consumption",
        )
    )

    # Safety thresholds
    collision_distance_threshold_m = 100.0
    dcpa_safety_threshold_m = 200.0
    tcpa_safety_threshold_s = 900.0

    # Compute per-target-ship safety margins and take the worst case
    all_dist_margins = []
    all_dcpa_margins = []
    all_tcpa_margins = []
    all_min_dist = []
    all_min_future_dcpa = []
    all_min_future_tcpa = []

    for k in range(1, num_target_ships + 1):
        _, _, dist_k = instance.get_ship_timeseries("OS0", f"dist_own_to_tar_{k}")
        _, _, dcpa_k = instance.get_ship_timeseries("OS0", f"dcpa_own_to_tar_{k}")
        _, _, tcpa_k = instance.get_ship_timeseries("OS0", f"tcpa_own_to_tar_{k}")

        dist_k = np.asarray(dist_k, dtype=float)
        dcpa_k = np.asarray(dcpa_k, dtype=float)
        tcpa_k = np.asarray(tcpa_k, dtype=float)

        min_dist_k = float(np.min(dist_k)) if dist_k.size else np.inf

        future_mask = np.isfinite(tcpa_k) & (tcpa_k > 0.0)
        if np.any(future_mask):
            min_future_dcpa_k = float(np.min(dcpa_k[future_mask]))
            min_future_tcpa_k = float(np.min(tcpa_k[future_mask]))
        else:
            min_future_dcpa_k = np.inf
            min_future_tcpa_k = np.inf

        all_min_dist.append(min_dist_k)
        all_min_future_dcpa.append(min_future_dcpa_k)
        all_min_future_tcpa.append(min_future_tcpa_k)

        all_dist_margins.append(_capped_margin(min_dist_k, collision_distance_threshold_m))
        all_dcpa_margins.append(_capped_margin(min_future_dcpa_k, dcpa_safety_threshold_m))
        all_tcpa_margins.append(_capped_margin(min_future_tcpa_k, tcpa_safety_threshold_s))

    # Worst-case across all target ships
    dist_margin = min(all_dist_margins) if all_dist_margins else 1.0
    dcpa_margin = min(all_dcpa_margins) if all_dcpa_margins else 1.0
    tcpa_margin = min(all_tcpa_margins) if all_tcpa_margins else 1.0

    metrics = {
        "max_abs_e_ct": float(np.max(np.abs(e_ct))) if e_ct.size else float("inf"),
        "mean_abs_e_ct": float(np.mean(np.abs(e_ct))) if e_ct.size else float("inf"),
        "fuel_consumption": fuel_consumption,
        "reached_end": reached_end,
        "collision": collision,
        "min_dist": min(all_min_dist) if all_min_dist else float("inf"),
        "min_future_dcpa": min(all_min_future_dcpa) if all_min_future_dcpa else float("inf"),
        "min_future_tcpa": min(all_min_future_tcpa) if all_min_future_tcpa else float("inf"),
        "dist_margin": dist_margin,
        "dcpa_margin": dcpa_margin,
        "tcpa_margin": tcpa_margin,
    }

    # Per-target details
    for k in range(num_target_ships):
        suffix = k + 1
        metrics[f"min_dist_tar_{suffix}"] = all_min_dist[k]
        metrics[f"min_future_dcpa_tar_{suffix}"] = all_min_future_dcpa[k]
        metrics[f"min_future_tcpa_tar_{suffix}"] = all_min_future_tcpa[k]
        metrics[f"dist_margin_tar_{suffix}"] = all_dist_margins[k]
        metrics[f"dcpa_margin_tar_{suffix}"] = all_dcpa_margins[k]
        metrics[f"tcpa_margin_tar_{suffix}"] = all_tcpa_margins[k]

    # Objective: smaller is worse / more dangerous; collision dominates
    objective = (
        3.0 * dist_margin
        + 2.0 * dcpa_margin
        + 1.5 * tcpa_margin
        + 0.25 * min(metrics["max_abs_e_ct"] / 300.0, 1.0)
        + 0.01 * fuel_consumption
    )

    if collision:
        objective -= 10.0

    if (not reached_end) and (not collision):
        objective -= 1.0

    metrics["objective"] = float(objective)
    return metrics


# ═════════════════════════════════════════════════════════════
# Ax parameter spaces
# ═════════════════════════════════════════════════════════════
def build_parameter_space_direct(target_ship_ids, max_intermediate_wps=3):
    """Ax search space for direct-spawn scenarios (BO controls spawn + route)."""
    parameters = []

    for ts_id in target_ship_ids:
        prefix = ts_id.lower()

        parameters.extend([
            {
                "name": f"{prefix}_spawn_north",
                "type": "range",
                "bounds": [6500.0, 8500.0],
                "value_type": "float",
            },
            {
                "name": f"{prefix}_spawn_east",
                "type": "range",
                "bounds": [-1500.0, 1500.0],
                "value_type": "float",
            },
            {
                "name": f"{prefix}_spawn_yaw_angle_deg",
                "type": "range",
                "bounds": [120.0, 240.0],
                "value_type": "float",
            },
            {
                "name": f"{prefix}_spawn_forward_speed",
                "type": "range",
                "bounds": [0.0, 8.0],
                "value_type": "float",
            },
            {
                "name": f"{prefix}_wp_count",
                "type": "range",
                "bounds": [0, max_intermediate_wps],
                "value_type": "int",
            },
            {
                "name": f"{prefix}_end_north",
                "type": "range",
                "bounds": [-500.0, 1500.0],
                "value_type": "float",
            },
            {
                "name": f"{prefix}_end_east",
                "type": "range",
                "bounds": [-1500.0, 1500.0],
                "value_type": "float",
            },
        ])

        for i in range(1, max_intermediate_wps + 1):
            parameters.extend([
                {
                    "name": f"{prefix}_wp_{i}_north",
                    "type": "range",
                    "bounds": [0.0, 8000.0],
                    "value_type": "float",
                },
                {
                    "name": f"{prefix}_wp_{i}_east",
                    "type": "range",
                    "bounds": [-1500.0, 1500.0],
                    "value_type": "float",
                },
            ])

        for i in range(1, max_intermediate_wps + 2):
            parameters.append(
                {
                    "name": f"{prefix}_leg_speed_{i}",
                    "type": "range",
                    "bounds": [0.5, 8.0],
                    "value_type": "float",
                }
            )

    return parameters


def build_parameter_space_trafficgen(encounters, encounter_settings_path, max_intermediate_wps=3):
    """Ax search space: trafficgen encounter params + intermediate WPs.

    Bounds for relativeSpeed and vectorTime are read from encounter_settings.json
    so they stay aligned automatically.
    """
    from orchestrator.scenario_config import load_encounter_settings, ENCOUNTER_TYPE_TO_KEY

    settings = load_encounter_settings(encounter_settings_path)
    vector_range = settings["vectorRange"]
    rel_speed_map = settings["relativeSpeed"]

    parameters = []
    for enc in encounters:
        ts_id = enc["id"].lower()
        enc_key = ENCOUNTER_TYPE_TO_KEY[enc["encounter_type"]]
        speed_bounds = rel_speed_map[enc_key]

        parameters.extend([
            {
                "name": f"{ts_id}_relative_speed",
                "type": "range",
                "bounds": speed_bounds,
                "value_type": "float",
            },
            {
                "name": f"{ts_id}_vector_time",
                "type": "range",
                "bounds": vector_range,
                "value_type": "float",
            },
            {
                "name": f"{ts_id}_wp_count",
                "type": "range",
                "bounds": [0, max_intermediate_wps],
                "value_type": "int",
            },
        ])

        for i in range(1, max_intermediate_wps + 1):
            parameters.extend([
                {
                    "name": f"{ts_id}_wp_{i}_north",
                    "type": "range",
                    "bounds": [0.0, 2000.0],
                    "value_type": "float",
                },
                {
                    "name": f"{ts_id}_wp_{i}_east",
                    "type": "range",
                    "bounds": [-1000.0, 1000.0],
                    "value_type": "float",
                },
            ])

        for i in range(1, max_intermediate_wps + 2):
            parameters.append({
                "name": f"{ts_id}_leg_speed_{i}",
                "type": "range",
                "bounds": [0.5, 8.0],
                "value_type": "float",
            })

    return parameters


# ═════════════════════════════════════════════════════════════
# Ax optimization loop
# ═════════════════════════════════════════════════════════════
def run_ax_optimization(
    experiment_name,
    parameter_space,
    evaluate_fn,
    total_trials=15,
    num_sobol_trials=5,
    random_seed=7,
):
    """Execute the full Ax optimization loop.

    Parameters
    ----------
    experiment_name : str
        Name for the Ax experiment.
    parameter_space : list[dict]
        Ax parameter definitions (from build_parameter_space_*).
    evaluate_fn : callable
        ``evaluate_fn(parameters) -> (raw_data, metrics)`` where *raw_data*
        is ``{"objective": (value, sem)}`` and *metrics* is a flat dict.
    total_trials, num_sobol_trials, random_seed : int
        Standard Ax loop settings.
    """
    if num_sobol_trials < 0:
        raise ValueError("num_sobol_trials must be >= 0")
    if num_sobol_trials > total_trials:
        raise ValueError("num_sobol_trials cannot be greater than total_trials")

    ax_client = AxClient(random_seed=random_seed)
    ax_client.create_experiment(
        name=experiment_name,
        parameters=parameter_space,
        objectives={"objective": ObjectiveProperties(minimize=True)},
        choose_generation_strategy_kwargs={
            "num_initialization_trials": num_sobol_trials,
        },
    )

    history = []

    for _ in range(total_trials):
        parameters, trial_index = ax_client.get_next_trial()

        try:
            raw_data, metrics = evaluate_fn(parameters)
            ax_client.complete_trial(trial_index=trial_index, raw_data=raw_data)
        except Exception as exc:
            ax_client.log_trial_failure(trial_index=trial_index)
            print(f"Trial {trial_index} failed: {exc}")
            print(traceback.format_exc())
            metrics = {"objective": float("inf"), "error": str(exc)}

        history.append(
            {
                "trial_index": trial_index,
                "parameters": parameters,
                "metrics": metrics,
            }
        )

    completed_trials = sum(1 for item in history if "error" not in item["metrics"])
    failed_trials = len(history) - completed_trials
    print(f"Completed trials: {completed_trials}, Failed trials: {failed_trials}")

    best_result = ax_client.get_best_parameters()
    if best_result is None:
        return ax_client, None, history

    best_parameters, _ = best_result
    return ax_client, best_parameters, history


# ═════════════════════════════════════════════════════════════
# Replay best trial
# ═════════════════════════════════════════════════════════════
def replay_best_trial(
    config_preparer_fn,
    target_ship_ids,
    best_parameters,
    root,
    save_dir,
    best_anim_path,
):
    """Re-run the best BO solution with plots and animation.

    Parameters
    ----------
    config_preparer_fn : callable
        ``config_preparer_fn(parameters) -> config`` — builds a ready-to-run
        YAML config dict from the given BO parameters.
    target_ship_ids : list[str]
        IDs of target ships (e.g. ``["TS1", "TS2"]``).
    best_parameters : dict
        Best BO parameters to replay.
    root : Path
        Project root for simulation.
    save_dir : Path
        Directory for saved animation files.
    best_anim_path : Path
        Full path for the best-trial animation file.
    """
    config = config_preparer_fn(best_parameters)
    instance = run_simulation(config, root)

    save_dir.mkdir(parents=True, exist_ok=True)

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
        save_path=best_anim_path,
        writer_fps=20,
        palette=None,
        blit=True,
        ship_scale=1.0,
    )

    instance.PlotFleetTrajectory(mode="quick", ship_scale=1.0)

    # OS keys (full ship model with machinery)
    key_group_list = [
        ["OS0.north"],
        ["OS0.east"],
        ["OS0.forward_speed", "OS0.next_wp_speed", "OS0.total_ship_speed"],
        ["OS0.yaw_angle_rad", "OS0.yaw_angle_ref_rad"],
        ["OS0.rudder_angle_deg"],
        ["OS0.e_ct"],
        ["OS0.shaft_speed_rpm", "OS0.shaft_speed_cmd_rpm"],
        ["OS0.throttle_cmd"],
    ]

    # Per-target-ship keys (simplified ship model)
    for ts_id in target_ship_ids:
        key_group_list.extend([
            [f"{ts_id}.north"],
            [f"{ts_id}.east"],
            [f"{ts_id}.forward_speed", f"{ts_id}.next_wp_speed", f"{ts_id}.total_ship_speed"],
            [f"{ts_id}.yaw_angle_rad", f"{ts_id}.yaw_angle_ref_rad"],
            [f"{ts_id}.rudder_angle_deg"],
            [f"{ts_id}.e_ct"],
            [f"{ts_id}.thrust_force"],
        ])

    instance.JoinPlotTimeSeries(
        list(reversed(key_group_list)),
        create_title=False,
        legend=True,
        show_instance_name=False,
        show_separately=False,
        show=True,
        mode="quick",
    )

    return instance


# ═════════════════════════════════════════════════════════════
# Parameter trimming & result saving
# ═════════════════════════════════════════════════════════════
def _trim_parameters(parameters):
    """Return a copy of parameters with unused intermediate WP entries removed."""
    ts_wp_counts = {}
    for key, value in parameters.items():
        m = re.match(r"(ts\d+)_wp_count$", key)
        if m:
            ts_wp_counts[m.group(1)] = int(value)

    trimmed = {}
    for key, value in parameters.items():
        m = re.match(r"(ts\d+)_wp_(\d+)_(north|east)$", key)
        if m:
            prefix, wp_idx = m.group(1), int(m.group(2))
            if wp_idx > ts_wp_counts.get(prefix, 0):
                continue

        m = re.match(r"(ts\d+)_leg_speed_(\d+)$", key)
        if m:
            prefix, leg_idx = m.group(1), int(m.group(2))
            if leg_idx > ts_wp_counts.get(prefix, 0) + 1:
                continue

        trimmed[key] = value
    return trimmed


def save_results(ax_client, best_parameters, history, results_path, trim=True):
    """Save the best parameters and full trial history to JSON.

    Parameters
    ----------
    trim : bool
        If True, remove unused intermediate WP entries from saved parameters.
    """
    _maybe_trim = _trim_parameters if trim else (lambda p: p)

    enriched_history = []
    for item in history:
        trial_index = int(item["trial_index"])
        enriched_item = dict(item)
        enriched_item["parameters"] = _maybe_trim(item["parameters"])

        try:
            trial = ax_client.get_trial(trial_index)
            enriched_item["trial_type"] = trial.generation_method_str
            enriched_item["trial_status"] = str(trial.status).split(".")[-1]
        except Exception as exc:
            print(f"Could not extract Ax metadata for trial {trial_index}: {exc}")
            enriched_item["trial_type"] = "unknown"
            enriched_item["trial_status"] = enriched_item.get("trial_status", "unknown")

        enriched_history.append(enriched_item)

    payload = {
        "best_parameters": _maybe_trim(best_parameters) if best_parameters is not None else None,
        "history": enriched_history,
    }

    with results_path.open("w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)


# ═════════════════════════════════════════════════════════════
# Ax trial summary table
# ═════════════════════════════════════════════════════════════
def print_trial_summary_table(ax_client):
    """Print a final Ax trial summary table with fixed-width columns."""
    try:
        df = ax_client.get_trials_data_frame()

        if df.empty:
            print("No trial data available.")
            return

        preferred_columns = [
            "trial_index",
            "trial_status",
            "generation_method",
            "objective",
        ]
        ordered_columns = [col for col in preferred_columns if col in df.columns]
        remaining_columns = [col for col in df.columns if col not in ordered_columns]
        df = df[ordered_columns + remaining_columns].copy()

        for col in df.columns:
            if df[col].dtype.kind in {"f", "i"}:
                if col == "trial_index":
                    df[col] = df[col].map(lambda x: f"{int(x)}" if not pd.isna(x) else "")
                else:
                    df[col] = df[col].map(
                        lambda x: f"{float(x):.3f}" if not pd.isna(x) else ""
                    )
            else:
                df[col] = df[col].fillna("").astype(str)

        max_widths = {
            "trial_index": 11,
            "trial_status": 14,
            "generation_method": 18,
            "objective": 12,
        }
        default_width = 14

        def shorten(text, width):
            text = str(text)
            if len(text) <= width:
                return text.ljust(width)
            return text[: width - 3] + "..."

        widths = {}
        for col in df.columns:
            target_width = max_widths.get(col, default_width)
            widths[col] = max(target_width, len(col))

        header = " | ".join(shorten(col, widths[col]) for col in df.columns)
        separator = "-+-".join("-" * widths[col] for col in df.columns)

        print("\nFinal Ax trial summary:")
        print(header)
        print(separator)

        for _, row in df.iterrows():
            line = " | ".join(
                shorten(row[col], widths[col]) for col in df.columns
            )
            print(line)

    except Exception as exc:
        print(f"Could not build Ax trial summary table: {exc}")
