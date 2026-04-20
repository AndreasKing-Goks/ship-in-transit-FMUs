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

## Get the config path
config_path = ROOT / "test_run" / "ast_iw_sampler_test" / "ast_iw_sampler_multi_ship_test_config.yaml"

## Get the save path for animation
save_path = ROOT / "saved_animation" / "ast_iw_sampler_multi_ship_test.gif"

with config_path.open("r", encoding="utf-8") as f:
    config = yaml.safe_load(f)
    
# =========================
# Spawn Requests
# =========================
# Spawn requests (Singapore Strait)  
own_ship = {
    "start_time"        : 0.0,
    "north_route"       : [0, 8000 , 10000 ],
    "east_route"        : [0, 8000 , 10000 ],
    "speed_setpoints"   : [0, 5, 3.5]
}
target_ship_1 = {
    "start_time"        : 0.0, 
    "north_route"       : [10000, 2000 , 0 ],
    "east_route"        : [0, 8000 , 10000 ],
    "speed_setpoints"   : [0, 5, 3.5]    
}
target_ship_2 = {
    "start_time"        : 0.0, 
    "north_route"       : [10000, 2000 , 0 ],
    "east_route"        : [5000, 5000 , 5000 ],
    "speed_setpoints"   : [0, 4, 2.5]    
}
target_ship_3 = {
    "start_time"        : 0.0, 
    "north_route"       : [5000, 5000 , 5000 ],
    "east_route"        : [10000, 2000 , 0 ],
    "speed_setpoints"   : [0, 4, 2.5]      
}
spawn_requests = {
    "OS0": own_ship,
    "TS1": target_ship_1,
    "TS2": target_ship_2,
    "TS3": target_ship_3,
}
    
# =========================
# Instantiate Co-simulation Wrapper
# =========================
# Instantiate
instance = ShipInTransitCoSimulation(config=config, ROOT=ROOT, spawn_requests=spawn_requests, IW_sampling_animated=True)


# =========================
# Simulate
# =========================
scope_angles_deg = {
    "TS1": {
        "sa"  : [5, 5, 10, 5, 0],
        "idx" : 0
        },
    "TS2": {
        "sa"  : [-5, -10, -15, 0, 0],
        "idx" : 0
        },
    "TS3": {
        "sa"  : [-10, -10, -5, 5, 0],
        "idx" : 0
        },
}

ship_ids = [key for key in scope_angles_deg.keys()]

while instance.time <= instance.stopTime:
    
    # Step the simulator
    instance.step() 
    
    # Own Ship Position and trigger zone radius
    north = instance.GetLastValue(
        slaveName=instance.ship_slave(prefix="OS0", block="SHIP_MODEL"),
        slaveVar="north"
    )
    east = instance.GetLastValue(
        slaveName=instance.ship_slave(prefix="OS0", block="SHIP_MODEL"),
        slaveVar="east"
    )
    pos = [north, east]
    
    os0_config = next(sc for sc in instance.ship_configs if sc.get("id") == "OS0")
    trigger_zone_rad_2 = os0_config["fmu_params"]["COLAV"]["danger_zone_radius"] ** 2
    
    for ship_config in instance.ship_configs:
        sid = ship_config.get("id")

        if sid not in ship_ids:
            continue

        # Target Ship Position
        test_north = instance.GetLastValue(
            slaveName=instance.ship_slave(prefix=sid, block="SHIP_MODEL"),
            slaveVar="north"
        )
        test_east = instance.GetLastValue(
            slaveName=instance.ship_slave(prefix=sid, block="SHIP_MODEL"),
            slaveVar="east"
        )
        test_pos = [test_north, test_east]

        dist_os_to_ts_2 = (pos[0] - test_pos[0])**2 + (pos[1] - test_pos[1])**2

        if dist_os_to_ts_2 < trigger_zone_rad_2:
            instance.SingleVariableManipulation(
                slaveName=instance.ship_slave(prefix=sid, block="MISSION_MANAGER"),
                slaveVar="inside_trigger_zone",
                value=True
            )
        else:
            instance.SingleVariableManipulation(
                slaveName=instance.ship_slave(prefix=sid, block="MISSION_MANAGER"),
                slaveVar="inside_trigger_zone",
                value=False
            )

        request_scope_angle = instance.GetLastValue(
            slaveName=instance.ship_slave(prefix=sid, block="MISSION_MANAGER"),
            slaveVar="request_scope_angle"
        )
        
        messages = instance.GetLastValue(
            slaveName=instance.ship_slave(prefix=sid, block="MISSION_MANAGER"),
            slaveVar="messages"
        )
        
        is_inside = instance.GetLastValue(
            slaveName=instance.ship_slave(prefix=sid, block="MISSION_MANAGER"),
            slaveVar="inside_trigger_zone"
        )

        if request_scope_angle:
            idx = scope_angles_deg[sid]["idx"]
            sa_list = scope_angles_deg[sid]["sa"]

            if idx < len(sa_list):
                instance.SingleVariableManipulation(
                    slaveName=instance.ship_slave(prefix=sid, block="MISSION_MANAGER"),
                    slaveVar="scope_angle_deg",
                    value=sa_list[idx]
                )

                scope_angles_deg[sid]["idx"] += 1
        
            print(f"time={instance.time:.2f}, sid={sid}")
            print(f"  OS pos        = {pos}")
            print(f"  TS pos        = {test_pos}")
            print(f"  dist2         = {dist_os_to_ts_2:.2f}")
            print(f"  trigger2      = {trigger_zone_rad_2:.2f}")
            print(f"  inside_zone   = {dist_os_to_ts_2 < trigger_zone_rad_2}")
            print(f"  request_scope = {request_scope_angle}")
            print(f"  idx           = {scope_angles_deg[sid]['idx']}")
            print(f"  messages      = {messages}")
            print("----")
    
    # Determined the termination flag
    if not instance.stop:
        instance.time += instance.stepSize
    else:
        break

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
        plot_inter_wp_roa=False,
        plot_inter_wp_proj=False,
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
    ["OS0.north"],
    ["OS0.east"],
    ["OS0.forward_speed", "OS0.next_wp_speed", "OS0.total_ship_speed"],
    ["OS0.yaw_angle_rad", "OS0.yaw_angle_ref_rad"],
    ["OS0.rudder_angle_deg"],
    ["OS0.e_ct"],
    ["OS0.shaft_speed_rpm", "OS0.shaft_speed_cmd_rpm"],
    ["OS0.throttle_cmd"],
    
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
    
    ## Target Ship(s)
    # Base results
    # ["TS1.north"],
    # ["TS1.east"],
    # ["TS1.forward_speed", "TS1.next_wp_speed", "TS1.total_ship_speed"],
    # ["TS1.yaw_angle_rad", "TS1.yaw_angle_ref_rad"],
    # ["TS1.rudder_angle_deg"],
    # ["TS1.e_ct"],
    # ["TS1.thrust_force"]
]

# Plot Time Series
instance.JoinPlotTimeSeries(list(reversed(key_group_list)),  create_title= False, legend= True, show_instance_name=False, show=True)