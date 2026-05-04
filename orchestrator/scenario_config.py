"""Scenario configuration: generic helpers for direct-spawn and trafficgen target-ship setups."""

import copy
import json
import math
import tempfile

import yaml
import numpy as np
from pathlib import Path

# trafficgen imports are optional — only required for the trafficgen approach
try:
    from trafficgen.ship_traffic_generator import generate_traffic_situations
    from trafficgen.marine_system_simulator import llh2flat
    from trafficgen.utils import knot_2_m_pr_s
    _HAS_TRAFFICGEN = True
except ImportError:
    _HAS_TRAFFICGEN = False


# ═════════════════════════════════════════════════════════════
# Shared helpers
# ═════════════════════════════════════════════════════════════
def load_base_config(config_path):
    """Load a baseline YAML scenario config."""
    with Path(config_path).open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def get_target_ship_ids(config):
    """Return the list of target ship IDs from the config (everything except OS*)."""
    return [s["id"] for s in config["ships"] if not s["id"].startswith("OS")]


# ═════════════════════════════════════════════════════════════
# Direct-spawn approach
# ═════════════════════════════════════════════════════════════
def apply_trial_parameters_direct(config, parameterization, target_ship_ids):
    """Apply Ax trial parameters to all target-ship spawns and routes (direct spawn)."""
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


def prepare_trial_config_direct(parameterization, config_path, target_ship_ids):
    """Load config → deep-copy → apply direct-spawn trial parameters."""
    config = copy.deepcopy(load_base_config(config_path))
    apply_trial_parameters_direct(config, parameterization, target_ship_ids)
    return config


# ═════════════════════════════════════════════════════════════
# Trafficgen approach
# ═════════════════════════════════════════════════════════════
ENCOUNTER_TYPE_TO_KEY = {
    "head-on":             "headOn",
    "overtaking-stand-on": "overtakingStandOn",
    "overtaking-give-way": "overtakingGiveWay",
    "crossing-give-way":   "crossingGiveWay",
    "crossing-stand-on":   "crossingStandOn",
}


def _require_trafficgen():
    if not _HAS_TRAFFICGEN:
        raise ImportError(
            "trafficgen is not installed. Install it to use the trafficgen approach."
        )


def load_encounter_settings(settings_path):
    """Load the encounter settings JSON."""
    with Path(settings_path).open("r", encoding="utf-8") as f:
        return json.load(f)


def build_situation_input_dict(parameterization, encounters, os_initial):
    """Build a trafficgen situation-input dict from encounter parameters."""
    enc_list = []
    for enc in encounters:
        ts_id = enc["id"].lower()
        enc_list.append({
            "desiredEncounterType": enc["encounter_type"],
            "relativeSpeed": float(parameterization[f"{ts_id}_relative_speed"]),
            "vectorTime": float(parameterization[f"{ts_id}_vector_time"]),
        })
    return {
        "title": "GENERATED_ENCOUNTER",
        "description": f"Generated encounter ({len(encounters)} TS)",
        "ownShip": {"initial": os_initial},
        "encounters": enc_list,
    }


def generate_situation(
    parameterization,
    encounters,
    os_initial,
    own_ship_path,
    target_ships_path,
    encounter_settings_path,
    max_retries=5,
):
    """Call trafficgen to produce a TrafficSituation from encounter parameters.

    trafficgen uses internal randomness (random future position for the target
    ship, random beta when unset, etc.).  The same parameters can therefore
    fail on one draw but succeed on the next.  We retry up to *max_retries*
    times before giving up.
    """
    _require_trafficgen()

    situation_dict = build_situation_input_dict(parameterization, encounters, os_initial)
    num_encounters = len(encounters)

    for attempt in range(1, max_retries + 1):
        with tempfile.TemporaryDirectory() as tmp_dir:
            sit_file = Path(tmp_dir) / "situation.json"
            sit_file.write_text(json.dumps(situation_dict), encoding="utf-8")

            situations = generate_traffic_situations(
                situations_data=Path(tmp_dir),
                own_ship_data=Path(own_ship_path),
                target_ships_data=Path(target_ships_path),
                settings_data=Path(encounter_settings_path),
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
    """Convert trafficgen output (lat/lon radians) to NED coordinates."""
    _require_trafficgen()

    os_wp0 = traffic_situation.own_ship.waypoints[0]
    lat_0 = os_wp0.position.lat    # already radians
    lon_0 = os_wp0.position.lon    # already radians

    ts_ned_list = []
    for ts in traffic_situation.target_ships:
        wps = []
        for wp in ts.waypoints:
            north, east, _ = llh2flat(
                wp.position.lat,
                wp.position.lon,
                lat_0, lon_0,
            )
            sog_ms = (
                wp.leg.sog
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
            "heading_deg": float(math.degrees(hdg_rad)),
            "waypoints": wps,
        })
    return ts_ned_list


def apply_trial_parameters_trafficgen(config, parameterization, ts_ned_data, encounters):
    """Set TS spawn & route from trafficgen output + intermediate WPs."""
    for idx, enc in enumerate(encounters):
        ts_id = enc["id"]
        ts_id_lower = ts_id.lower()
        ts_cfg = next(s for s in config["ships"] if s["id"] == ts_id)
        ts = ts_ned_data[idx]

        start = ts["waypoints"][0]
        end = ts["waypoints"][-1]

        ts_cfg["spawn"]["north"] = start["north"]
        ts_cfg["spawn"]["east"] = start["east"]
        ts_cfg["spawn"]["yaw_angle_deg"] = ts["heading_deg"]
        ts_cfg["spawn"]["forward_speed"] = 0

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


def prepare_trial_config_trafficgen(
    parameterization,
    config_path,
    encounters,
    os_initial,
    own_ship_path,
    target_ships_path,
    encounter_settings_path,
):
    """Generate situation → convert to NED → populate config (end-to-end)."""
    situation = generate_situation(
        parameterization, encounters, os_initial,
        own_ship_path, target_ships_path, encounter_settings_path,
    )
    ts_ned = convert_trafficgen_to_ned(situation)
    config = copy.deepcopy(load_base_config(config_path))
    apply_trial_parameters_trafficgen(config, parameterization, ts_ned, encounters)
    return config
