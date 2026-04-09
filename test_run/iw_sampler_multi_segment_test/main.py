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

# =========================
# Load the Configuration
# =========================
import yaml

## Get the config path
config_path = ROOT / "test_run" / "iw_sampler_multi_segment_test" / "iw_sampler_multi_segment_test.yaml"

## Get the save path for animation
save_path = ROOT / "saved_animation" / "iw_sampler_test.gif"

with config_path.open("r", encoding="utf-8") as f:
    config = yaml.safe_load(f)

# =========================
# Spawn Requests
# =========================
own_ship = {
    "start_time"        : 0.0,
    "north_route"       : [0, 10000, 0],
    "east_route"        : [0, 10000, 20000],
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
# # Start timer
# start_time = time.perf_counter()

# scope_angles_deg = [30, -30, -30, -15, -30, 0, 15, 30, 0]
scope_angles_deg = [30, 0, -45, -45, 0]
i = 0

# Initialize outside your timestep loop
last_idx = None
last_msg = None
last_prev = None
last_next = None

while instance.time <= instance.stopTime:

    if instance.time > 500e9:
        instance.SingleVariableManipulation(
            slaveName="OS0__MISSION_MANAGER",
            slaveVar="inside_trigger_zone",
            value=True
        )
        
        inside = instance.GetLastValue(
            slaveName="OS0__MISSION_MANAGER",
            slaveVar="inside_trigger_zone",
        )

    # Read outputs from the previous completed step
    request_scope_angle = instance.GetLastValue(
        slaveName="OS0__MISSION_MANAGER",
        slaveVar="request_scope_angle"
    )
    
    if request_scope_angle:
        instance.SingleVariableManipulation(
            slaveName="OS0__MISSION_MANAGER",
            slaveVar="scope_angle_deg",
            value=scope_angles_deg[i]
        )
        # print(f"  -> injecting scope angle {scope_angles_deg[i]} deg")
        i += 1

    instance.step()
    
    prev_wp_north   = instance.GetLastValue("OS0__MISSION_MANAGER", "prev_wp_north")
    prev_wp_east    = instance.GetLastValue("OS0__MISSION_MANAGER", "prev_wp_east")
    next_wp_north   = instance.GetLastValue("OS0__MISSION_MANAGER", "next_wp_north")
    next_wp_east    = instance.GetLastValue("OS0__MISSION_MANAGER", "next_wp_east")
    idx             = instance.GetLastValue("OS0__MISSION_MANAGER", "idx")
    msg             = str(instance.GetLastValue("OS0__MISSION_MANAGER", "messages"))
    
    # Only print if something changed
    if idx != last_idx or msg != last_msg:
        print("traj_index_", idx)
        print(f"msg : {msg}")
        print(f"prev: ({prev_wp_north:.1f}, {prev_wp_east:.1f})")
        print(f"next: ({next_wp_north:.1f}, {next_wp_east:.1f})")
        print("####")

        # Update stored values
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
        frame_step=2,
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