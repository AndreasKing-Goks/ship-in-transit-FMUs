from pathlib import Path
import sys
import os

# Workaround for OpenMP duplicate runtime
os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

# Ensure libcosim DLL is found
dll_dir = Path(sys.prefix) / "Lib" / "site-packages" / "libcosimpy" / "libcosimc"
os.add_dll_directory(str(dll_dir))

## PATH HELPER (OBLIGATORY)
# project root = two levels up from this file
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from orchestrator.sit_cosim import ShipInTransitCoSimulation
from orchestrator.scenario_config import (load_spawn_requests_bank_path, 
                                          load_base_config)

# =========================
# Handle paths and case index
# =========================
# Case Index
case_idx                        = 0

# Get the config path
config_path                     = ROOT / "EBASTv2_train" / "EBASTv2_train_2.yaml"

# Get the encounter settings path
encounter_settings_path         = ROOT / "EBASTv2_train" / "encounter_settings.json"

# Spawn requests bank path
spawn_requests_bank_path        = ROOT / "pre_generated_spawn_requests_bank" / "spawn_request_bank_1000.pkl"

# =========================
# # Instantiate Co-simulation Wrapper
# =========================
# Collect spawn requests
spawn_requests_bank             = load_spawn_requests_bank_path(spawn_requests_bank_path)
spawn_cases                     = spawn_requests_bank["cases"]    
case                            = spawn_cases[case_idx]
encounters                      = case["encounters"]

config                          = load_base_config(config_path=config_path)
spawn_requests                  = case["spawn_requests"]

print(spawn_requests)

# # Instantiate the RL-environment wrapper class
# instance = ShipInTransitCoSimulation(config=config, 
#                                      spawn_requests=spawn_requests, 
#                                      ROOT=ROOT)

# # =========================
# # Simulate
# # =========================
# instance.Simulate()

# # Print encounter type
# print("------------------------------------------------------------------")
# for ship_id in instance.ship_ids[1:]:   # Exclude the own ship
#     print(f"{ship_id}")
#     print(f"Encounter type  : {encounters[ship_id]['desiredEncounterType']}")
#     print(f"Vector time     : {encounters[ship_id]['vectorTime']}")
#     print(f"Bearing heading : {encounters[ship_id]['beta']}")
#     print(f"Relative speed  : {encounters[ship_id]['relativeSpeed']}")
#     print("------------------------------------------------------------------")

# print(spawn_requests)

# # =========================
# # Animation and Plot
# # =========================
# # Available formats:
# # - .mp4
# # - .gif
# # - .avi
# # - .mov

# # Animate Simulation
# instance.AnimateFleetTrajectory(
#         ship_ids=None,
#         show=True,
#         block=True,
#         mode="quick",
#         fig_width=10.0,
#         margin_frac=0.08,
#         equal_aspect=True,
#         interval_ms=20,
#         frame_step=10,
#         trail_len=50,
#         plot_routes=True,
#         plot_waypoints=True,
#         plot_roa=True,
#         plot_start_end=True,
#         plot_inter_wp_roa=False,
#         plot_inter_wp_proj=False,
#         with_labels=True,
#         precompute_ship_outlines=True,
#         writer_fps=20,
#         palette=None,
#         blit=True,
#         ship_scale=1.0
#     )

# # Plot Trajectory
# instance.PlotFleetTrajectory(mode="quick", ship_scale=1.0)

# # Plot Simulation Results
# key_group_list = [
#     ## Own Ship
#     # Base results
#     ["OS0.forward_speed", "OS0.next_wp_speed", "OS0.total_ship_speed"],
#     ["OS0.yaw_angle_rad", "OS0.yaw_angle_ref_rad"],
    
#     ## Target Ship(s)
#     # Base results
#     ["TS1.forward_speed", "TS1.next_wp_speed", "TS1.total_ship_speed"],
#     ["TS1.yaw_angle_rad", "TS1.yaw_angle_ref_rad"],
    
#     # Base results
#     ["TS2.forward_speed", "TS2.next_wp_speed", "TS2.total_ship_speed"],
#     ["TS2.yaw_angle_rad", "TS2.yaw_angle_ref_rad"],
# ]

# # Plot Time Series
# instance.JoinPlotTimeSeries(list(reversed(key_group_list)),  create_title= False, legend= True, show_instance_name=False, show=True)