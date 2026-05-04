from pathlib import Path
import sys
import os

# Ensure libcosim DLL is found
dll_dir = Path(sys.prefix) / "Lib" / "site-packages" / "libcosimpy" / "libcosimc"
os.add_dll_directory(str(dll_dir))

## PATH HELPER (OBLIGATORY)
# project root = two levels up from this file
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from orchestrator.sit_cosim import ShipInTransitCoSimulation
from orchestrator.scenario_config import load_base_config

# =========================
# Load the Configuration
# =========================
import yaml

## Get the config path
config_path = ROOT / "test_run" / "ast_iw_sampler_test" / "ast_iw_sampler_single_ship_test_config.yaml"

## Get the save path for animation
save_path   = ROOT / "saved_animation" / "ast_iw_sampler_single_ship_test.gif"   

# Get the configs
config      = load_base_config(config_path)

# =========================
# Spawn Requests
# =========================
own_ship = {
    "start_time"        : 0.0,
    "north_route"       : [0, 10000],
    "east_route"        : [0, 10000],
    "speed_setpoints"   : 4.0,
}
spawn_requests = {
    "OS0": own_ship,
}

# =========================
# Instantiate Co-simulation Wrapper
# =========================
# Instantiate
instance = ShipInTransitCoSimulation(config=config, ROOT=ROOT, spawn_requests=spawn_requests, IW_sampling_animated=True)

# =========================
# Simulate
# =========================

import math
import random

def generate_angle_deg(mu_deg=0.0, kappa=6.0, min_deg=-45.0, max_deg=45.0):
    """
    Generate an angle in degrees using a von Mises distribution,
    truncated to the interval [min_deg, max_deg].

    Parameters
    ----------
    mu_deg : float
        Mean direction in degrees.
    kappa : float
        Concentration parameter for von Mises.
        Higher values cluster more tightly around mu_deg.
    min_deg : float
        Minimum allowed angle in degrees.
    max_deg : float
        Maximum allowed angle in degrees.

    Returns
    -------
    float
        Random angle in degrees within [min_deg, max_deg].
    """
    mu_rad = math.radians(mu_deg)

    while True:
        angle_rad = random.vonmisesvariate(mu_rad, kappa)
        angle_deg = math.degrees(angle_rad)

        # Normalize to [-180, 180] for safety
        angle_deg = ((angle_deg + 180) % 360) - 180

        if min_deg <= angle_deg <= max_deg:
            return angle_deg


def generate_length(min_len=1500, max_len=4000, target_mean=2500, sigma=0.23):
    """
    Generate a length using a truncated log-normal distribution,
    bounded to [min_len, max_len].

    This is non-Gaussian and slightly favors medium-to-long values.

    Parameters
    ----------
    min_len : int or float
        Minimum allowed length.
    max_len : int or float
        Maximum allowed length.
    target_mean : float
        Approximate mean before truncation tuning.
    sigma : float
        Spread in log-space. Larger values create more right skew.

    Returns
    -------
    int
        Random length within [min_len, max_len].
    """
    # For a lognormal, mean = exp(mu + sigma^2 / 2)
    mu = math.log(target_mean) - (sigma ** 2) / 2

    while True:
        value = random.lognormvariate(mu, sigma)
        if min_len <= value <= max_len:
            return int(round(value))

# Initialize outside your timestep loop
last_idx = None
last_msg = None
last_prev = None
last_next = None
last_request_captain_intent = False

while instance.time <= instance.stopTime:

    request_captain_intent = instance.GetLastValue(
        slaveName="OS0__MISSION_MANAGER",
        slaveVar="request_captain_intent"
    )

    new_request = request_captain_intent and not last_request_captain_intent

    if new_request:
        angle = generate_angle_deg()
        length = generate_length()

        print(f"NEW SAMPLE -> angle={angle:.2f}, length={length}")

        instance.SingleVariableManipulation(
            slaveName="OS0__MISSION_MANAGER",
            slaveVar="scope_angle_deg",
            value=angle
        )
        instance.SingleVariableManipulation(
            slaveName="OS0__MISSION_MANAGER",
            slaveVar="scope_length",
            value=length
        )

    last_request_captain_intent = request_captain_intent

    instance.step()

    prev_wp_north = instance.GetLastValue("OS0__MISSION_MANAGER", "prev_wp_north")
    prev_wp_east  = instance.GetLastValue("OS0__MISSION_MANAGER", "prev_wp_east")
    next_wp_north = instance.GetLastValue("OS0__MISSION_MANAGER", "next_wp_north")
    next_wp_east  = instance.GetLastValue("OS0__MISSION_MANAGER", "next_wp_east")
    idx           = instance.GetLastValue("OS0__MISSION_MANAGER", "idx")
    msg           = str(instance.GetLastValue("OS0__MISSION_MANAGER", "messages"))

    if idx != last_idx or msg != last_msg:
        print("traj_index_", idx)
        print(f"msg : {msg}")
        print(f"prev: ({prev_wp_north:.1f}, {prev_wp_east:.1f})")
        print(f"next: ({next_wp_north:.1f}, {next_wp_east:.1f})")
        print("####")

        last_idx = idx
        last_msg = msg
        last_prev = (prev_wp_north, prev_wp_east)
        last_next = (next_wp_north, next_wp_east)

    if not instance.stop:
        instance.time += instance.stepSize
    else:
        break

for key in instance.IW_sampling_anim_data["OS0"].keys():
    print(f"===== frame:{key}")
    print("active_path            :", instance.IW_sampling_anim_data["OS0"][key]["active_path"])
    print("sampled_inter_wps      :", instance.IW_sampling_anim_data["OS0"][key]["sampled_inter_wps"])
    print("sampled_inter_wp_projs :", instance.IW_sampling_anim_data["OS0"][key]["sampled_inter_wp_projs"])
    print("=====")

# =========================
# Animation and Plot
# =========================
# Available formats:
# - .mp4
# - .gif
# - .avi
# - .mov

# Animate Simulation
instance.AnimateFleetTrajectory(
        ship_ids=None,
        show=True,
        block=True,
        mode="quick",
        fig_width=10.0,
        margin_frac=0.08,
        equal_aspect=True,
        interval_ms=20,
        frame_step=10,
        trail_len=50,
        plot_routes=True,
        plot_waypoints=True,
        plot_roa=True,
        plot_start_end=True,
        plot_inter_wp_roa=True,
        with_labels=True,
        precompute_ship_outlines=True,
        # save_path=save_path,
        writer_fps=20,
        palette=None,
        blit=True,
        ship_scale=1.0
    )

# Plot Trajectory
instance.PlotFleetTrajectory(mode="quick", ship_scale=1.0)

# Plot Simulation Results
key_group_list = [
    ## Own Ship
    # Base results
    # ["OS0.north"],
    # ["OS0.east"],
    # ["OS0.forward_speed", "OS0.next_wp_speed", "OS0.total_ship_speed"],
    # ["OS0.yaw_angle_rad", "OS0.yaw_angle_ref_rad"],
    # ["OS0.rudder_angle_deg"],
    # ["OS0.e_ct"],
    
    # Waypoints
    ["OS0.prev_wp_north"],
    ["OS0.prev_wp_east"],
    ["OS0.next_wp_north"],
    ["OS0.next_wp_east"],
    
    # # For non-single ship simulation only
    # ["OS0.new_throttle_cmd"],
    # ["OS0.new_rudder_angle_deg"],
    # ["OS0.colav_rud_ang_increment"],
    # ["OS0.beta_own_to_tar_1"],
    # ["OS0.tcpa_own_to_tar_1"],
    # ["OS0.dcpa_own_to_tar_1"],
    # ["OS0.dist_own_to_tar_1"],
    # ["OS0.rr_own_to_tar_1"],
    
    # # For environment load-enabled simulation only
    # ["OS0.current_speed"],
    # ["OS0.current_direction_deg"],
    # ["OS0.wind_speed"],
    # ["OS0.wind_direction_deg"],
    
]

# Plot Time Series
instance.JoinPlotTimeSeries(list(reversed(key_group_list)),  create_title= False, legend= True, show_instance_name=False, show=True)