import copy
import json
import math
import yaml
import numpy as np
from pathlib import Path
import sys
import os
import tempfile
import traceback
from ax.service.ax_client import AxClient, ObjectiveProperties
import pandas as pd

# Ensure libcosim DLL is found
dll_dir = Path(sys.prefix) / "Lib" / "site-packages" / "libcosimpy" / "libcosimc"
os.add_dll_directory(str(dll_dir))

# Project root = two levels up from this file
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from orchestrator.sit_cosim import ShipInTransitCoSimulation

# trafficgen imports
from trafficgen.ship_traffic_generator import generate_traffic_situations
from trafficgen.marine_system_simulator import llh2flat
from trafficgen.utils import knot_2_m_pr_s


# ─── Paths ───────────────────────────────────────────────────
CONFIG_PATH = Path(__file__).with_name("single_target_ship_ho.yaml")
SAVE_DIR = ROOT / "saved_animation"
BEST_ANIM_PATH = SAVE_DIR / "open_sea_one_ts_best_ax.mp4"
RESULTS_PATH = Path(__file__).with_name("ax_results.json")
OWN_SHIP_PATH = ROOT / "data" / "ownship" / "ownship.json"
TARGET_SHIPS_PATH = ROOT / "data" / "targetships"
ENCOUNTER_SETTINGS_PATH = Path(__file__).with_name("encounter_settings.json")


# ─── Encounter-type key mapping ──────────────────────────────
ENCOUNTER_TYPE_TO_KEY = {
    "head-on":             "headOn",
    "overtaking-stand-on": "overtakingStandOn",
    "overtaking-give-way": "overtakingGiveWay",
    "crossing-give-way":   "crossingGiveWay",
    "crossing-stand-on":   "crossingStandOn",
}


# ─── Fixed scenario decisions (Step 1) ──────────────────────
ENCOUNTERS = [
    {"id": "TS1", "encounter_type": "head-on"},
    # {"id": "TS2", "encounter_type": "crossing-give-way"},
]

OS_INITIAL = {
    "position": {"lat": 58.763449, "lon": 10.490654},
    "sog": 10.0,     # knots (≈5.14 m/s, consistent with config OS route speed)
    "cog": 0.0,      # degrees
    "heading": 0.0,   # degrees
    "navStatus": "Under way using engine",
}


# ═════════════════════════════════════════════════════════════
# Config loading
# ═════════════════════════════════════════════════════════════
def load_base_config():
    """Load the baseline YAML scenario config."""
    with CONFIG_PATH.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def load_encounter_settings():
    """Load the encounter settings JSON."""
    with ENCOUNTER_SETTINGS_PATH.open("r", encoding="utf-8") as f:
        return json.load(f)


# ═════════════════════════════════════════════════════════════
# trafficgen integration (Steps 4–5)
# ═════════════════════════════════════════════════════════════
def build_situation_input_dict(parameterization, encounters):
    """Build a trafficgen situation-input dict from BO encounter params."""
    enc_list = []
    for enc in encounters:
        ts_id = enc["id"].lower()
        enc_list.append({
            "desiredEncounterType": enc["encounter_type"],
            "relativeSpeed": float(parameterization[f"{ts_id}_relative_speed"]),
            "vectorTime": float(parameterization[f"{ts_id}_vector_time"]),
        })
    return {
        "title": "BO_ENCOUNTER",
        "description": f"BO-generated encounter ({len(encounters)} TS)",
        "ownShip": {"initial": OS_INITIAL},
        "encounters": enc_list,
    }


def generate_situation(parameterization, encounters, max_retries=5):
    """Call trafficgen to produce a TrafficSituation from BO encounter params.

    trafficgen uses internal randomness (random future position for the target
    ship, random beta when unset, etc.).  The same Ax parameters can therefore
    fail on one draw but succeed on the next.  We retry up to *max_retries*
    times before giving up.
    """
    situation_dict = build_situation_input_dict(parameterization, encounters)
    num_encounters = len(encounters)

    for attempt in range(1, max_retries + 1):
        # At the moment we are not saving the generated output files and instead using a temporary directory.
        with tempfile.TemporaryDirectory() as tmp_dir:
            sit_file = Path(tmp_dir) / "situation.json"
            sit_file.write_text(json.dumps(situation_dict), encoding="utf-8")

            situations = generate_traffic_situations(
                situations_data=Path(tmp_dir),
                own_ship_data=OWN_SHIP_PATH,
                target_ships_data=TARGET_SHIPS_PATH,
                settings_data=ENCOUNTER_SETTINGS_PATH,
            )

        if not situations:
            raise RuntimeError("trafficgen produced no traffic situations")

        situation = situations[0]

        if situation.target_ships and len(situation.target_ships) >= num_encounters:
            return situation

        if attempt < max_retries:
            print(
                f"  trafficgen attempt {attempt}/{max_retries} produced "
                f"{len(situation.target_ships) if situation.target_ships else 0}"
                f"/{num_encounters} target ships — retrying..."
            )

    raise RuntimeError(
        f"trafficgen could not produce {num_encounters} target ship(s) after "
        f"{max_retries} attempts (encounter parameters may be infeasible)"
    )


def convert_trafficgen_to_ned(traffic_situation):
    os_wp0 = traffic_situation.own_ship.waypoints[0]
    lat_0 = os_wp0.position.lat    # already radians
    lon_0 = os_wp0.position.lon    # already radians

    ts_ned_list = []
    for ts in traffic_situation.target_ships:
        wps = []
        for wp in ts.waypoints:
            north, east, _ = llh2flat(
                wp.position.lat,    # already radians
                wp.position.lon,    # already radians
                lat_0, lon_0,
            )
            sog_ms = (
                wp.leg.sog          # already m/s
                if (wp.leg is not None and wp.leg.sog is not None)
                else 0.0
            )
            wps.append({
                "north": float(north),
                "east": float(east),
                "speed_ms": float(sog_ms),
            })

        hdg_rad = (
            float(ts.initial.heading)
            if (ts.initial and ts.initial.heading is not None)
            else 0.0
        )
        ts_ned_list.append({
            "heading_deg": float(math.degrees(hdg_rad)),  # convert rad → deg
            "waypoints": wps,
        })
    return ts_ned_list


# ═════════════════════════════════════════════════════════════
# Config population (Step 6)
# ═════════════════════════════════════════════════════════════
def apply_trial_parameters(config, parameterization, ts_ned_data, encounters):
    """Set TS spawn & route from trafficgen output + BO intermediate WPs."""
    for idx, enc in enumerate(encounters):
        ts_id = enc["id"]
        ts_id_lower = ts_id.lower()
        ts_cfg = next(s for s in config["ships"] if s["id"] == ts_id)
        ts = ts_ned_data[idx]

        start = ts["waypoints"][0]
        end = ts["waypoints"][-1]

        # Spawn from trafficgen start position + heading
        ts_cfg["spawn"]["north"] = start["north"]
        ts_cfg["spawn"]["east"] = start["east"]
        ts_cfg["spawn"]["yaw_angle_deg"] = ts["heading_deg"]
        ts_cfg["spawn"]["forward_speed"] = 0  # ship accelerates to route speed

        # Route: start → (BO intermediates) → end
        wp_count = int(parameterization[f"{ts_id_lower}_wp_count"])

        route_north = [start["north"]]
        route_east = [start["east"]]
        route_speed = []

        for i in range(1, wp_count + 1):
            route_north.append(float(parameterization[f"{ts_id_lower}_wp_{i}_north"]))
            route_east.append(float(parameterization[f"{ts_id_lower}_wp_{i}_east"]))
            route_speed.append(float(parameterization[f"{ts_id_lower}_leg_speed_{i}"]))

        route_north.append(end["north"])
        route_east.append(end["east"])
        route_speed.append(float(parameterization[f"{ts_id_lower}_leg_speed_{wp_count + 1}"]))

        ts_cfg["route"] = {
            "north": route_north,
            "east": route_east,
            "speed": route_speed,
        }

    return config


# ═════════════════════════════════════════════════════════════
# Full trial config preparation (Steps 4→5→6)
# ═════════════════════════════════════════════════════════════
def prepare_trial_config(parameterization, encounters):
    """Generate situation → convert to NED → populate config."""
    situation = generate_situation(parameterization, encounters)
    ts_ned = convert_trafficgen_to_ned(situation)
    config = copy.deepcopy(load_base_config())
    apply_trial_parameters(config, parameterization, ts_ned, encounters)
    return config


# ═════════════════════════════════════════════════════════════
# Ax parameter space (Steps 2 + 3)
# ═════════════════════════════════════════════════════════════
def build_parameter_space(encounters, max_intermediate_wps=3):
    """Ax search space: trafficgen encounter params + intermediate WPs.

    Bounds for relativeSpeed and vectorTime are read from encounter_settings.json
    so they stay aligned automatically.
    """
    settings = load_encounter_settings()
    vector_range = settings["vectorRange"]
    rel_speed_map = settings["relativeSpeed"]

    parameters = []
    for enc in encounters:
        ts_id = enc["id"].lower()
        enc_key = ENCOUNTER_TYPE_TO_KEY[enc["encounter_type"]]
        speed_bounds = rel_speed_map[enc_key]

        # ── trafficgen encounter params ──
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
            # ── intermediate waypoint count ──
            {
                "name": f"{ts_id}_wp_count",
                "type": "range",
                "bounds": [0, max_intermediate_wps],
                "value_type": "int",
            },
        ])

        # Intermediate waypoint positions (absolute NED coordinates)
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

        # Leg speeds (one per leg = wp_count + 1)
        for i in range(1, max_intermediate_wps + 2):
            parameters.append({
                "name": f"{ts_id}_leg_speed_{i}",
                "type": "range",
                "bounds": [0.5, 8.0],
                "value_type": "float",
            })

    return parameters


# ═════════════════════════════════════════════════════════════
# Simulation & metrics (Step 7)
# ═════════════════════════════════════════════════════════════
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
    _, _, e_ct = instance.get_ship_timeseries("OS0", "e_ct")
    e_ct = np.asarray(e_ct, dtype=float)

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

    collision_distance_threshold_m = 100.0
    dcpa_safety_threshold_m = 200.0
    tcpa_safety_threshold_s = 900.0

    min_dist = float(np.min(dist)) if dist.size else np.inf
    future_mask = np.isfinite(tcpa) & (tcpa > 0.0)

    if np.any(future_mask):
        min_future_dcpa = float(np.min(dcpa[future_mask]))
        min_future_tcpa = float(np.min(tcpa[future_mask]))
    else:
        min_future_dcpa = np.inf
        min_future_tcpa = np.inf

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
# Ax trial evaluation (Step 8)
# ═════════════════════════════════════════════════════════════
def evaluate_trial(parameterization, encounters):
    """Run one Ax trial: generate → convert → simulate → metrics."""
    config = prepare_trial_config(parameterization, encounters)
    instance = run_simulation(config)
    metrics = compute_trial_metrics(instance)
    return {"objective": (metrics["objective"], 0.0)}, metrics


def run_ax_optimization(encounters, total_trials=15, num_sobol_trials=5, random_seed=7, max_intermediate_wps=3):
    """Execute the full Ax optimization loop."""
    if num_sobol_trials < 0:
        raise ValueError("num_sobol_trials must be >= 0")
    if num_sobol_trials > total_trials:
        raise ValueError("num_sobol_trials cannot exceed total_trials")

    ax_client = AxClient(random_seed=random_seed)
    ax_client.create_experiment(
        name="open_sea_one_ts_ho_bo_stg",
        parameters=build_parameter_space(encounters, max_intermediate_wps),
        objectives={"objective": ObjectiveProperties(minimize=True)},
        choose_generation_strategy_kwargs={
            "num_initialization_trials": num_sobol_trials,
        },
    )

    history = []
    for _ in range(total_trials):
        parameters, trial_index = ax_client.get_next_trial()
        try:
            raw_data, metrics = evaluate_trial(parameters, encounters)
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

    done = sum(1 for h in history if "error" not in h["metrics"])
    print(f"Completed trials: {done}, Failed trials: {len(history) - done}")

    best_result = ax_client.get_best_parameters()
    if best_result is None:
        return ax_client, None, history

    best_parameters, _ = best_result
    return ax_client, best_parameters, history


# ═════════════════════════════════════════════════════════════
# Replay & reporting
# ═════════════════════════════════════════════════════════════
def replay_best_trial(best_parameters, encounters):
    """Re-run the best BO solution with plots and animation."""
    config = prepare_trial_config(best_parameters, encounters)
    instance = run_simulation(config)

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


def save_results(ax_client, best_parameters, history):
    """Save the best parameters and full trial history to JSON."""
    enriched_history = []
    for item in history:
        trial_index = int(item["trial_index"])
        enriched_item = dict(item)
        try:
            trial = ax_client.get_trial(trial_index)
            enriched_item["trial_type"] = trial.generation_method_str
            enriched_item["trial_status"] = str(trial.status).split(".")[-1]
        except Exception as exc:
            print(f"Could not extract Ax metadata for trial {trial_index}: {exc}")
            enriched_item["trial_type"] = "unknown"
            enriched_item["trial_status"] = enriched_item.get(
                "trial_status", "unknown"
            )
        enriched_history.append(enriched_item)

    payload = {
        "best_parameters": best_parameters,
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

        for col in df.columns:
            if df[col].dtype.kind in {"f", "i"}:
                if col == "trial_index":
                    df[col] = df[col].map(
                        lambda x: f"{int(x)}" if not pd.isna(x) else ""
                    )
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


# ═════════════════════════════════════════════════════════════
# Main
# ═════════════════════════════════════════════════════════════
def main():
    """Run BO, persist results, and optionally replay the best trial."""
    num_sobol_trials = 2
    num_bo_trials = 2
    total_trials = num_sobol_trials + num_bo_trials
    replay_best = True

    print(
        f"Running Ax optimization with {num_sobol_trials} Sobol trials "
        f"and {num_bo_trials} BO trials (total={total_trials})."
    )

    ax_client, best_parameters, history = run_ax_optimization(
        encounters=ENCOUNTERS,
        total_trials=total_trials,
        num_sobol_trials=num_sobol_trials,
        random_seed=7,
        max_intermediate_wps=1,
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
        replay_best_trial(best_parameters, ENCOUNTERS)


if __name__ == "__main__":
    main()