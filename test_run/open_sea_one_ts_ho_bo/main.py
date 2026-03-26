import copy
import json
import yaml
import numpy as np
from pathlib import Path
import sys
import os
import traceback
from ax.service.ax_client import AxClient, ObjectiveProperties

# Ensure libcosim DLL is found
dll_dir = Path(sys.prefix) / "Lib" / "site-packages" / "libcosimpy" / "libcosimc"
os.add_dll_directory(str(dll_dir))

# project root = two levels up from this file
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from orchestrator.sit_cosim import ShipInTransitCoSimulation


CONFIG_PATH = Path(__file__).with_name("single_target_ship_ho.yaml")
SAVE_DIR = ROOT / "saved_animation"
BEST_ANIM_PATH = SAVE_DIR / "open_sea_one_ts_best_ax.mp4"
RESULTS_PATH = Path(__file__).with_name("ax_results.json")


def load_base_config():
    """Load the baseline YAML scenario config for BO trials."""
    with CONFIG_PATH.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def set_fmu_param(config, ship_id, block_name, param_name, value):
    """Set one FMU parameter for a specific ship in the config dict."""
    for ship_cfg in config["ships"]:
        if ship_cfg.get("id") == ship_id:
            ship_cfg["fmu_params"][block_name][param_name] = value
            return
    raise KeyError(f"Could not find ship_id={ship_id}")


def apply_trial_parameters(config, parameterization, max_intermediate_wps=3):
    """Apply Ax trial parameters to the target-ship spawn and route."""
    ts1_cfg = next(ship for ship in config["ships"] if ship["id"] == "TS1")

    spawn_north = float(parameterization["ts1_spawn_north"])
    spawn_east = float(parameterization["ts1_spawn_east"])
    spawn_yaw = float(parameterization["ts1_spawn_yaw_angle_deg"])
    spawn_speed = float(parameterization["ts1_spawn_forward_speed"])
    wp_count = int(parameterization["ts1_wp_count"])

    ts1_cfg["spawn"]["north"] = spawn_north
    ts1_cfg["spawn"]["east"] = spawn_east
    ts1_cfg["spawn"]["yaw_angle_deg"] = spawn_yaw
    ts1_cfg["spawn"]["forward_speed"] = spawn_speed

    route_north = [spawn_north]
    route_east = [spawn_east]
    route_speed = []

    for i in range(1, wp_count + 1):
        route_north.append(float(parameterization[f"ts1_wp_{i}_north"]))
        route_east.append(float(parameterization[f"ts1_wp_{i}_east"]))
        route_speed.append(float(parameterization[f"ts1_leg_speed_{i}"]))

    route_north.append(float(parameterization["ts1_end_north"]))
    route_east.append(float(parameterization["ts1_end_east"]))
    route_speed.append(float(parameterization[f"ts1_leg_speed_{wp_count + 1}"]))

    ts1_cfg["route"] = {
        "north": route_north,
        "east": route_east,
        "speed": route_speed,
    }

    return config


def build_parameter_space(max_intermediate_wps=3):
    """Define the Ax search space for target-ship scenario parameters."""
    parameters = [
        {
            "name": "ts1_spawn_north",
            "type": "range",
            "bounds": [6500.0, 8500.0],
            "value_type": "float",
        },
        {
            "name": "ts1_spawn_east",
            "type": "range",
            "bounds": [-1500.0, 1500.0],
            "value_type": "float",
        },
        {
            "name": "ts1_spawn_yaw_angle_deg",
            "type": "range",
            "bounds": [120.0, 240.0],
            "value_type": "float",
        },
        {
            "name": "ts1_spawn_forward_speed",
            "type": "range",
            "bounds": [0.0, 8.0],
            "value_type": "float",
        },
        {
            "name": "ts1_wp_count",
            "type": "range",
            "bounds": [0, max_intermediate_wps],
            "value_type": "int",
        },
        {
            "name": "ts1_end_north",
            "type": "range",
            "bounds": [-500.0, 1500.0],
            "value_type": "float",
        },
        {
            "name": "ts1_end_east",
            "type": "range",
            "bounds": [-1500.0, 1500.0],
            "value_type": "float",
        },
    ]

    for i in range(1, max_intermediate_wps + 1):
        parameters.extend(
            [
                {
                    "name": f"ts1_wp_{i}_north",
                    "type": "range",
                    "bounds": [0.0, 8000.0],
                    "value_type": "float",
                },
                {
                    "name": f"ts1_wp_{i}_east",
                    "type": "range",
                    "bounds": [-1500.0, 1500.0],
                    "value_type": "float",
                },
            ]
        )

    # Number of route legs = number of intermediate WPs + 1
    for i in range(1, max_intermediate_wps + 2):
        parameters.append(
            {
                "name": f"ts1_leg_speed_{i}",
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


def compute_trial_metrics(instance):
    """Extract safety-based trial metrics and combine them into one BO objective."""
    # Path-following performance
    _, _, e_ct = instance.get_ship_timeseries("OS0", "e_ct")
    e_ct = np.asarray(e_ct, dtype=float)

    # COLAV outputs for the OS against TS1
    _, _, dist = instance.get_ship_timeseries("OS0", "dist_own_to_tar_1")
    _, _, dcpa = instance.get_ship_timeseries("OS0", "dcpa_own_to_tar_1")
    _, _, tcpa = instance.get_ship_timeseries("OS0", "tcpa_own_to_tar_1")

    dist = np.asarray(dist, dtype=float)
    dcpa = np.asarray(dcpa, dtype=float)
    tcpa = np.asarray(tcpa, dtype=float)

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

    min_dist = float(np.min(dist)) if dist.size else np.inf

    # Only future encounter points are relevant.
    future_mask = np.isfinite(tcpa) & (tcpa > 0.0)

    if np.any(future_mask):
        min_future_dcpa = float(np.min(dcpa[future_mask]))
        min_future_tcpa = float(np.min(tcpa[future_mask]))
    else:
        min_future_dcpa = np.inf
        min_future_tcpa = np.inf

    # Normalize each risk term against its threshold.
    # Lower objective should mean more dangerous for Ax(minimize=True).
    dist_margin = _capped_margin(min_dist, collision_distance_threshold_m)
    dcpa_margin = _capped_margin(min_future_dcpa, dcpa_safety_threshold_m)
    tcpa_margin = _capped_margin(min_future_tcpa, tcpa_safety_threshold_s)

    metrics = {
        "max_abs_e_ct": float(np.max(np.abs(e_ct))) if e_ct.size else float("inf"),
        "mean_abs_e_ct": float(np.mean(np.abs(e_ct))) if e_ct.size else float("inf"),
        "fuel_consumption": fuel_consumption,
        "reached_end": reached_end,
        "collision": collision,
        "min_dist": min_dist,
        "min_future_dcpa": min_future_dcpa,
        "min_future_tcpa": min_future_tcpa,
        "dist_margin": dist_margin,
        "dcpa_margin": dcpa_margin,
        "tcpa_margin": tcpa_margin,
    }

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


def evaluate_trial(parameterization):
    """Run one Ax trial from suggested parameters and return metric data."""
    base_config = load_base_config()
    trial_config = copy.deepcopy(base_config)
    apply_trial_parameters(trial_config, parameterization)

    instance = run_simulation(trial_config)
    metrics = compute_trial_metrics(instance)

    raw_data = {
        "objective": (metrics["objective"], 0.0),
    }

    return raw_data, metrics


def run_ax_optimization(total_trials=15, random_seed=7):
    """Execute the full Ax optimization loop across multiple trials."""
    ax_client = AxClient(random_seed=random_seed)

    ax_client.create_experiment(
        name="open_sea_one_ts_ho_bo",
        parameters=build_parameter_space(),
        objectives={
            "objective": ObjectiveProperties(minimize=True),
        },
    )

    history = []

    for _ in range(total_trials):
        parameters, trial_index = ax_client.get_next_trial()

        try:
            raw_data, metrics = evaluate_trial(parameters)
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


def replay_best_trial(best_parameters):
    """Re-run the best BO solution with plots and animation enabled."""
    base_config = load_base_config()
    best_config = copy.deepcopy(base_config)
    apply_trial_parameters(best_config, best_parameters)

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

    key_group_list = [
        ["OS0.north"],
        ["OS0.east"],
        ["OS0.forward_speed", "OS0.next_wp_speed", "OS0.total_ship_speed"],
        ["OS0.yaw_angle_rad", "OS0.yaw_angle_ref_rad"],
        ["OS0.rudder_angle_deg"],
        ["OS0.e_ct"],
        ["OS0.shaft_speed_rpm", "OS0.shaft_speed_cmd_rpm"],
        ["OS0.throttle_cmd"],
        ["TS1.north"],
        ["TS1.east"],
        ["TS1.forward_speed", "TS1.next_wp_speed", "TS1.total_ship_speed"],
        ["TS1.yaw_angle_rad", "TS1.yaw_angle_ref_rad"],
        ["TS1.rudder_angle_deg"],
        ["TS1.e_ct"],
        ["TS1.thrust_force"],
    ]

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


def save_results(best_parameters, history):
    """Save the best parameters and full trial history to JSON."""
    payload = {
        "best_parameters": best_parameters,
        "history": history,
    }
    with RESULTS_PATH.open("w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)


def print_trial_summary_table(ax_client):
    """Print a final Ax trial summary table."""
    try:
        df = ax_client.get_trials_data_frame()

        if df.empty:
            print("No trial data available.")
            return

        # Optional: keep the most relevant columns first if they exist.
        preferred_columns = [
            "trial_index",
            "trial_status",
            "generation_method",
            "objective",
        ]
        ordered_columns = [col for col in preferred_columns if col in df.columns]
        remaining_columns = [col for col in df.columns if col not in ordered_columns]
        df = df[ordered_columns + remaining_columns]

        print("\nFinal Ax trial summary:")
        print(df.to_string(index=False))
    except Exception as exc:
        print(f"Could not build Ax trial summary table: {exc}")


def main():
    """Run BO, persist results, and optionally replay the best trial."""
    total_trials = 3
    replay_best = True

    ax_client, best_parameters, history = run_ax_optimization(
        total_trials=total_trials,
        random_seed=7,
    )

    save_results(best_parameters, history)
    print_trial_summary_table(ax_client)

    if best_parameters is None:
        print("No valid Ax trials were completed. Check trial errors in history.")
        return

    print("Best parameters found:")
    for key, value in best_parameters.items():
        print(f"  {key}: {value}")

    if replay_best:
        replay_best_trial(best_parameters)


if __name__ == "__main__":
    main()