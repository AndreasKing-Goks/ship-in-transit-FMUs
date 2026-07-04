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

# Write into case description file
case_description_path           = ROOT / "pre_generated_spawn_requests_bank" / "animation" / "case_description.txt"
case_description_path.parent.mkdir(parents=True, exist_ok=True)
case_description_path.write_text("", encoding="utf-8")

# Saved animation folder path
saved_animation_folder_path     = ROOT / "pre_generated_spawn_requests_bank" / "animation"
saved_animation_folder_path.mkdir(parents=True, exist_ok=True)

# Case Index
case_idxs                       = range(10)

# Print and save encounter description
description_lines = []
description_lines.append("==================================================================")

for case_idx in case_idxs:
    description_lines.append(f"CASE {case_idx}")
    description_lines.append("==================================================================")
    
    # =========================
    # Handle paths and case index
    # =========================
    # Get the config path
    config_path                     = ROOT / "EBASTv2_train" / "EBASTv2_train_2.yaml"

    # Get the encounter settings path
    encounter_settings_path         = ROOT / "EBASTv2_train" / "encounter_settings.json"

    # Spawn requests bank path
    spawn_requests_bank_path        = ROOT / "pre_generated_spawn_requests_bank" / "spawn_request_bank_1000.pkl"

    # Get the save path for animation
    saved_animation_path            = ROOT / "pre_generated_spawn_requests_bank" / "animation" / f"case_{case_idx}.gif"
    
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

    # Instantiate the RL-environment wrapper class
    instance = ShipInTransitCoSimulation(config=config, 
                                        spawn_requests=spawn_requests, 
                                        ROOT=ROOT)

    # =========================
    # Simulate
    # =========================
    instance.Simulate()
    
    i = 0

    # Print encounter description
    for ship_id in instance.ship_ids[1:]:   # Exclude the own ship
        description_lines.append(f"{ship_id}")
        description_lines.append(f"Encounter type  : {encounters[ship_id]['desiredEncounterType']}")
        description_lines.append(f"Vector time     : {encounters[ship_id]['vectorTime']}")
        description_lines.append(f"Bearing heading : {encounters[ship_id]['beta']}")
        description_lines.append(f"Relative speed  : {encounters[ship_id]['relativeSpeed']}")
        
        if i == (len(instance.ship_ids)-1):
            description_lines.append("==================================================================")
        else:
            description_lines.append("------------------------------------------------------------------")
            
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
            show=False,
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
            plot_inter_wp_roa=False,
            plot_inter_wp_proj=False,
            with_labels=True,
            precompute_ship_outlines=True,
            save_path=saved_animation_path,
            writer_fps=20,
            palette=None,
            blit=True,
            ship_scale=1.0
        )

# Compile and write the description text
description_text = "\n".join(description_lines)

with open(case_description_path, "a", encoding="utf-8") as f:
    f.write(description_text)
    f.write("\n\n")