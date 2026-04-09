import copy
import json
import yaml
import numpy as np
from pathlib import Path
import sys
import os
import re
import traceback
from ax.service.ax_client import AxClient, ObjectiveProperties
import pandas as pd

# Ensure libcosim DLL is found
dll_dir = Path(sys.prefix) / "Lib" / "site-packages" / "libcosimpy" / "libcosimc"
os.add_dll_directory(str(dll_dir))

# project root = two levels up from this file
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from orchestrator.sit_cosim import ShipInTransitCoSimulation


CONFIG_PATH = Path(__file__).with_name("two_target_ship_ho.yaml")
SAVE_DIR = ROOT / "saved_animation"
BEST_ANIM_PATH = SAVE_DIR / "open_sea_two_ts_best_ax.mp4"
RESULTS_PATH = Path(__file__).with_name("ax_results.json")


def load_base_config():
    """Load the baseline YAML scenario config for BO trials."""
    with CONFIG_PATH.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def get_target_ship_ids(config):
    """Return the list of target ship IDs from the config (everything except OS*)."""
    return [s["id"] for s in config["ships"] if not s["id"].startswith("OS")]


def apply_trial_parameters(config, parameterization, target_ship_ids):
    """Apply Ax trial parameters to all target-ship spawns and routes."""
    for ts_id in target_ship_ids:
        prefix = ts_id.lower()
        ts_cfg = next(ship for ship in config["ships"] if ship["id"] == ts_id)

        spawn_north = float(parameterization[f"{prefix}_spawn_north"])
        spawn_east = float(parameterization[f"{prefix}_spawn_east"])
        spawn_yaw = float(parameterization[f"{prefix}_spawn_yaw_angle_deg"])
        spawn_speed = float(parameterization[f"{prefix}_spawn_forward_speed"])
        wp_count = int(parameterization[f"{prefix}_wp_count"])

        ts_cfg["spawn"]["north"] = spawn_north
        ts_cfg["spawn"]["east"] = spawn_east
        ts_cfg["spawn"]["yaw_angle_deg"] = spawn_yaw
        ts_cfg["spawn"]["forward_speed"] = spawn_speed

        route_north = [spawn_north]
        route_east = [spawn_east]
        route_speed = []

        for i in range(1, wp_count + 1):
            route_north.append(float(parameterization[f"{prefix}_wp_{i}_north"]))
            route_east.append(float(parameterization[f"{prefix}_wp_{i}_east"]))
            route_speed.append(float(parameterization[f"{prefix}_leg_speed_{i}"]))

        route_north.append(float(parameterization[f"{prefix}_end_north"]))
        route_east.append(float(parameterization[f"{prefix}_end_east"]))
        route_speed.append(float(parameterization[f"{prefix}_leg_speed_{wp_count + 1}"]))

        ts_cfg["route"] = {
            "north": route_north,
            "east": route_east,
            "speed": route_speed,
        }

    return config


def build_parameter_space(target_ship_ids, max_intermediate_wps=3):
    """Define the Ax search space for all target-ship scenario parameters."""
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

        # Number of route legs = number of intermediate WPs + 1
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


def build_instance(config):
    """Create one co-simulation instance from a prepared config."""
    return ShipInTransitCoSimulation(config=config, ROOT=ROOT)


def run_simulation(config):
    """Build and run one simulation trial, then return the instance."""
    instance = build_instance(config)
    instance.Simulate()
    return instance


def _capped_margin(value, threshold):
    """Return a normalized safety margin in [0, 1], where 0 is worst."""
    if not np.isfinite(value):
        return 1.0
    if threshold <= 0:
        return 1.0
    return float(np.clip(value / threshold, 0.0, 1.0))


def compute_trial_metrics(instance, num_target_ships):
    """Extract safety-based trial metrics and combine them into one BO objective."""
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

    # Objective interpretation:
    # - smaller is worse / more dangerous
    # - collision should dominate everything
    # - negative TCPA is ignored by construction
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


def evaluate_trial(parameterization, target_ship_ids):
    """Run one Ax trial from suggested parameters and return metric data."""
    base_config = load_base_config()
    trial_config = copy.deepcopy(base_config)
    apply_trial_parameters(trial_config, parameterization, target_ship_ids)

    instance = run_simulation(trial_config)
    metrics = compute_trial_metrics(instance, num_target_ships=len(target_ship_ids))

    raw_data = {
        "objective": (metrics["objective"], 0.0),
    }

    return raw_data, metrics


def run_ax_optimization(target_ship_ids, total_trials=15, num_sobol_trials=5, random_seed=7):
    """Execute the full Ax optimization loop across multiple trials."""
    if num_sobol_trials < 0:
        raise ValueError("num_sobol_trials must be >= 0")
    if num_sobol_trials > total_trials:
        raise ValueError("num_sobol_trials cannot be greater than total_trials")

    ax_client = AxClient(random_seed=random_seed)

    ax_client.create_experiment(
        name="open_sea_one_ts_ho_bo",
        parameters=build_parameter_space(target_ship_ids),
        objectives={
            "objective": ObjectiveProperties(minimize=True),
        },
        choose_generation_strategy_kwargs={
            "num_initialization_trials": num_sobol_trials,
        },
    )

    history = []

    for _ in range(total_trials):
        parameters, trial_index = ax_client.get_next_trial()

        try:
            raw_data, metrics = evaluate_trial(parameters, target_ship_ids)
            ax_client.complete_trial(trial_index=trial_index, raw_data=raw_data)
        except Exception as exc:
            ax_client.log_trial_failure(trial_index=trial_index)
            print(f"Trial {trial_index} failed: {exc}")
            print(traceback.format_exc())
            metrics = {
                "objective": float("inf"),
                "error": str(exc),
            }

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


def replay_best_trial(best_parameters, target_ship_ids):
    """Re-run the best BO solution with plots and animation enabled."""
    base_config = load_base_config()
    best_config = copy.deepcopy(base_config)
    apply_trial_parameters(best_config, best_parameters, target_ship_ids)

    instance = run_simulation(best_config)

    SAVE_DIR.mkdir(parents=True, exist_ok=True)

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
        save_path=BEST_ANIM_PATH,
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


def _trim_parameters(parameters):
    """Return a copy of parameters with unused intermediate WP entries removed."""
    # Discover per-ship wp_count values
    ts_wp_counts = {}
    for key, value in parameters.items():
        m = re.match(r"(ts\d+)_wp_count$", key)
        if m:
            ts_wp_counts[m.group(1)] = int(value)

    trimmed = {}
    for key, value in parameters.items():
        # Match intermediate waypoint parameters: e.g. ts1_wp_2_north
        m = re.match(r"(ts\d+)_wp_(\d+)_(north|east)$", key)
        if m:
            prefix, wp_idx = m.group(1), int(m.group(2))
            if wp_idx > ts_wp_counts.get(prefix, 0):
                continue

        # Match leg speed parameters: e.g. ts1_leg_speed_3
        m = re.match(r"(ts\d+)_leg_speed_(\d+)$", key)
        if m:
            prefix, leg_idx = m.group(1), int(m.group(2))
            if leg_idx > ts_wp_counts.get(prefix, 0) + 1:
                continue

        trimmed[key] = value
    return trimmed


def save_results(ax_client, best_parameters, history):
    """Save the best parameters and full trial history to JSON."""
    enriched_history = []

    for item in history:
        trial_index = int(item["trial_index"])
        enriched_item = dict(item)
        enriched_item["parameters"] = _trim_parameters(item["parameters"])  # <-- trim here

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
        "best_parameters": _trim_parameters(best_parameters) if best_parameters is not None else None,  # <-- trim here
        "history": enriched_history,
    }

    with RESULTS_PATH.open("w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)


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

        # Format floats for readability.
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

        # Fixed-width layout settings.
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
            return (text[: width - 3] + "...")

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


def main():
    """Run BO, persist results, and optionally replay the best trial."""
    num_sobol_trials = 2
    num_bo_trials = 2
    total_trials = num_sobol_trials + num_bo_trials
    replay_best = True

    base_config = load_base_config()
    target_ship_ids = get_target_ship_ids(base_config)

    print(
        f"Running Ax optimization with {num_sobol_trials} Sobol trials "
        f"and {num_bo_trials} BO trials (total={total_trials}), "
        f"target ships: {target_ship_ids}."
    )

    ax_client, best_parameters, history = run_ax_optimization(
        target_ship_ids=target_ship_ids,
        total_trials=total_trials,
        num_sobol_trials=num_sobol_trials,
        random_seed=7,
    )

    save_results(ax_client, best_parameters, history)
    print_trial_summary_table(ax_client)

    if best_parameters is None:
        print("No valid Ax trials were completed. Check trial errors in history.")
        return

    print("Best parameters found:")
    for key, value in best_parameters.items():
        print(f"  {key}: {value}")

    if replay_best:
        replay_best_trial(best_parameters, target_ship_ids)


if __name__ == "__main__":
    main()