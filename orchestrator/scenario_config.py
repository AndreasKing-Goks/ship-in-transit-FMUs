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
    from trafficgen.marine_system_simulator import llh2flat, flat2llh
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


def build_situation_input_dict(encounters, own_ship_initial):
    """
        Build a trafficgen situation-input dict from encounter parameters.
    """
    enc_list = []

    for ts_id, enc in encounters.items():
        enc_list.append({
            "desiredEncounterType": enc["desiredEncounterType"],
            "relativeSpeed": float(enc["relativeSpeed"]),
            "vectorTime": float(enc["vectorTime"]),
            # optional:
            **({"beta": float(enc["beta"])} if "beta" in enc else {})
        })

    return {
        "title": "GENERATED_ENCOUNTER",
        "description": f"Generated encounter ({len(enc_list)} TS)",
        "ownShip": {"initial": own_ship_initial},
        "encounters": enc_list,
    }
    
def generate_traffic_gen_situation(
    encounters,
    own_ship_initial,
    own_ship_desc,
    target_ships_desc,
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
    
    for attempt in range (1, max_retries + 1):
    
        with tempfile.TemporaryDirectory() as tmp:
            tmp_dir = Path(tmp)

            situations_dir = tmp_dir / "situations"
            target_dir = tmp_dir / "target_ships"
            situations_dir.mkdir()
            target_dir.mkdir()

            own_ship_file = tmp_dir / "own_ship.json"

            situation_file = situations_dir / "situation.json"
            situation_data = build_situation_input_dict(encounters, own_ship_initial)

            with open(situation_file, "w", encoding="utf-8") as f:
                json.dump(situation_data, f, indent=4)

            with open(own_ship_file, "w", encoding="utf-8") as f:
                json.dump(own_ship_desc, f, indent=4)

            for i, target_desc in enumerate(target_ships_desc):
                target_file = target_dir / f"target_ship_{i}.json"
                with open(target_file, "w", encoding="utf-8") as f:
                    json.dump(target_desc, f, indent=4)

            situations = generate_traffic_situations(
                situations_data=situations_dir,
                own_ship_data=own_ship_file,
                target_ships_data=target_dir,
                settings_data=Path(encounter_settings_path),
            )
            
        if not situations:
            raise RuntimeError("trafficgen produced no traffic situations")
        
        situation = situations[0]

        num_encounters = len(encounters)
        if situation.target_ships and len(situation.target_ships) >= num_encounters:
            return situation
        
        if attempt < max_retries:
            print(
                f"  trafficgen attempt {attempt}/{max_retries} produced "
                f"{len(situation.target_ships) if situation.target_ships else 0}"
                f"/{num_encounters} target ships — retrying..."
            )

    return situations

def get_own_ship_description(own_ship_config):
    # Get the own ship descriptions
    length      = own_ship_config["fmu_params"]["SHIP_MODEL"]["length_of_ship"]
    width       = own_ship_config["fmu_params"]["SHIP_MODEL"]["width_of_ship"]
    height      = own_ship_config["fmu_params"]["SHIP_MODEL"]["front_above_water_height"]
    
    sogMax      = own_ship_config.get("sogMax", 13.0)       # If not specified, assume max speed of ground is 13 m/s
    mmsi        = own_ship_config.get("mmsi", 100000001)    # If not specified, assume mmsi as 100000001
    name        = own_ship_config.get("id")
    shipType    = own_ship_config.get("shipType", "Cargo")  # If not specified, assume ship type as Cargo
    
    own_ship_desc       = {
        "dimensions":
            {
                "length":length,
                "width":width,
                "height":height,
            },
        "sogMax":sogMax*1.94384449,   # Convert m/s to knot
        "mmsi":mmsi,
        "name":name,
        "shipType":shipType
    }
    return own_ship_desc

def get_target_ships_description(target_ship_configs):
    target_ships_desc = []
    
    for ts_config in target_ship_configs:
        length      = ts_config["fmu_params"]["SHIP_MODEL"]["length_of_ship"]
        width       = ts_config["fmu_params"]["SHIP_MODEL"]["width_of_ship"]
        height      = ts_config["fmu_params"]["SHIP_MODEL"]["front_above_water_height"]
        shipType    = ts_config.get("shipType", "Cargo")  # If not specified, assume ship type as Cargo
        sogMax      = ts_config.get("sogMax", 13.0)       # If not specified, assume max speed over ground is 13 m/s
        
        data = {
            "dimensions":
                {
                    "length":length,
                    "width":width,
                    "height":height,
                },
            "sogMax":sogMax*1.94384449,   # Convert m/s to knot
            "shipType":shipType
            }
        
        target_ships_desc.append(data)
    
    return target_ships_desc

def convert_own_ship_initial_ned_to_llh(own_ship_initial_ned, origin_lat_deg=58.763449, origin_lon_deg=10.490654):
    """
        Convert own ship initial (NED coordinates) to lat/lon radians and velocity unit from m/s to knot.
    """
    north   = float(own_ship_initial_ned["position"]["north"])
    east    = float(own_ship_initial_ned["position"]["east"])
    sog     = float(own_ship_initial_ned["sog"])

    lat0_rad = math.radians(origin_lat_deg)
    lon0_rad = math.radians(origin_lon_deg)

    lat_rad, lon_rad, _ = flat2llh(
        north,
        east,
        0.0,
        lat0_rad,
        lon0_rad,
    )

    own_ship_initial_llh = copy.deepcopy(own_ship_initial_ned)
    own_ship_initial_llh["position"] = {
        "lat": math.degrees(lat_rad),
        "lon": math.degrees(lon_rad),
    }
    own_ship_initial_llh["sog"] = sog * 1.94384449  # Convert m/s to knot

    return own_ship_initial_llh

def convert_trafficgen_to_ned(traffic_situations, encounters):
    """
        Convert trafficgen output (lat/lon radians) to NED coordinates.
    """
    _require_trafficgen()
    
    situations_ned = {}
    
    ## Own Ship
    # Get origin latitude and longitude
    os_wp0 = traffic_situations.own_ship.waypoints[0]
    lat_0 = os_wp0.position.lat                         # already radians
    lon_0 = os_wp0.position.lon                         # already radians
    
    # Own Ship waypoints
    os_wps = []
    for wp in traffic_situations.own_ship.waypoints:
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
        os_wps.append({
            "north": float(north),
            "east": float(east),
            "speed_ms": float(sog_ms),
        })
    os_hdg_rad = (
            float(traffic_situations.own_ship.initial.heading)
            if (traffic_situations.own_ship.initial and traffic_situations.own_ship.initial.heading is not None)
            else 0.0
        )
    
    # Update the situation for Own Ship
    os_situation            = {
        "heading_deg": float(math.degrees(os_hdg_rad)),
        "waypoints": os_wps,
    }
    situations_ned["OS0"]   = os_situation

    ## Target Ships
    # Initial index
    idx     = 0
    ts_ids  = list(encounters.keys())
    for ts in traffic_situations.target_ships:
        # Target Ships' waypoints
        ts_wps = []
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
            ts_wps.append({
                "north": float(north),
                "east": float(east),
                "speed_ms": float(sog_ms),
            })
        
        # Target Ships' heading
        ts_hdg_rad = (
            float(ts.initial.heading)
            if (ts.initial and ts.initial.heading is not None)
            else 0.0
        )
        
        # Compute the Target Ships' situation
        ts_situation        = {
            "heading_deg": float(math.degrees(ts_hdg_rad)),
            "waypoints": ts_wps,
        }
        
        # Increment the target ship index
        ts_id               = ts_ids[idx]
        idx                += 1
        
        # Update the situation based on the index
        situations_ned[ts_id]   = ts_situation
    
    return situations_ned

def apply_trial_parameters_trafficgen(situations_ned):
    """Set ship asset spawn & route from trafficgen output + intermediate WPs."""
    
    # Initial containers
    spawn_requests  = {}
    
    for ship_id in situations_ned.keys():
        # Compute the spawn items
        situation   = situations_ned[ship_id]

        start       = situation["waypoints"][0]
        end         = situation["waypoints"][-1]

        route_north = [start["north"], end["north"]]
        route_east  = [start["east"], end["east"]]
        route_speed = [0.0, end["speed_ms"]]
        # route_speed = [start["speed_ms"], end["speed_ms"]]
        
        # Get the spawn requests
        spawn = {
            "start_time": 0.0,
            "north_route": route_north,
            "east_route": route_east,
            "yaw_angle_deg": situation["heading_deg"],
            "speed_setpoint": route_speed
        }
        
        # Update the spawn request
        spawn_requests[ship_id]   = spawn

    return spawn_requests

def prepare_config_and_spawn_requests_with_traffic_gen(
    own_ship_initial_ned,
    encounters,
    config_path,
    encounter_settings_path,
):
    """Generate situation → convert to NED → populate config (end-to-end)."""
    # First get the initial config
    temp_config         = load_base_config(config_path)
        
    # Upack the config file
    ship_configs        = temp_config["ships"]
    own_ship_config     = ship_configs[0]
    target_ship_configs = ship_configs[1:]
    
    # Get the own ship and target ships descriptions
    own_ship_desc       = get_own_ship_description(own_ship_config)
    target_ships_desc   = get_target_ships_description(target_ship_configs)
    
    # Convert the own ship initial information from NED to latitude-longitude
    own_ship_initial    = convert_own_ship_initial_ned_to_llh(own_ship_initial_ned)
    
    # Get the situation and convert it again to NED situation
    traffic_situations = generate_traffic_gen_situation(
        encounters, own_ship_initial, own_ship_desc, target_ships_desc, encounter_settings_path,
    )
    situations_ned = convert_trafficgen_to_ned(traffic_situations, encounters)
    
    # Compute the config and spawn requests
    config              = copy.deepcopy(load_base_config(config_path))
    spawn_requests      = apply_trial_parameters_trafficgen(situations_ned)
    
    return config, spawn_requests
