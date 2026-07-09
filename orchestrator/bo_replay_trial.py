"""
Replay a specific BO trial from results with animation and plots.

This is a generalized replay script that works for any BO scenario.
Call it from any test_run directory with the scenario config.

Usage:
    python ../../orchestrator/bo_replay_trial.py --scenario two_target_ship_ho.yaml --trial 5
    python ../../orchestrator/bo_replay_trial.py --scenario two_target_ship_ho.yaml --trial 5 --save
    python ../../orchestrator/bo_replay_trial.py --scenario two_target_ship_ho.yaml --best
    python ../../orchestrator/bo_replay_trial.py --scenario two_target_ship_ho.yaml --list
    
Or from the test_run directory with simpler alias:
    alias bo_replay='python ../../orchestrator/bo_replay_trial.py'
    bo_replay --scenario two_target_ship_ho.yaml --trial 5
"""

from pathlib import Path
import sys
import json
import argparse
import os
import pandas as pd
import copy

# Ensure libcosim DLL is found
dll_dir = Path(sys.prefix) / "Lib" / "site-packages" / "libcosimpy" / "libcosimc"
os.add_dll_directory(str(dll_dir))

# Get root from this script's location (orchestrator folder)
ORCHESTRATOR_DIR = Path(__file__).resolve().parent
ROOT = ORCHESTRATOR_DIR.parent
sys.path.insert(0, str(ROOT))

from orchestrator.bo_core import replay_best_trial, prepare_spawn_requests_trafficgen
from orchestrator.scenario_config import load_base_config, load_encounter_settings


def get_scenario_info(scenario_path, current_dir=None):
    """Load scenario configuration and locate results in the scenario directory."""
    scenario_path = Path(scenario_path)
    
    # If relative path, resolve from current_dir; if absolute, use as-is
    if not scenario_path.is_absolute():
        if current_dir:
            scenario_path = Path(current_dir) / scenario_path
        else:
            scenario_path = Path.cwd() / scenario_path
    
    if not scenario_path.exists():
        raise FileNotFoundError(f"Scenario config not found: {scenario_path}")
    
    # Results are in the same directory as the config
    scenario_dir = scenario_path.parent
    results_path = scenario_dir / "ax_results.json"
    csv_path = scenario_dir / "ax_results_metrics.csv"
    encounter_settings_path = scenario_dir / "encounter_settings.json"
    save_dir = ROOT / "saved_animation"
    
    if not results_path.exists():
        raise FileNotFoundError(f"Results file not found: {results_path}")
    
    return {
        "config_path": scenario_path,
        "results_path": results_path,
        "csv_path": csv_path,
        "encounter_settings_path": encounter_settings_path,
        "save_dir": save_dir,
        "scenario_dir": scenario_dir,
    }


def load_csv_and_json(paths):
    """Load both CSV and JSON results."""
    if not paths["csv_path"].exists():
        print(f"CSV file not found: {paths['csv_path']}")
        print("Run main.py first to generate results.")
        return None, None
    
    if not paths["results_path"].exists():
        print(f"JSON file not found: {paths['results_path']}")
        return None, None
    
    df = pd.read_csv(paths["csv_path"])
    with open(paths["results_path"], "r") as f:
        data = json.load(f)
    
    return df, data


def parse_encounters_from_results(data, config_path, encounter_settings_path):
    """Extract ENCOUNTERS from the first successful trial in results."""
    # Load config to determine number of target ships
    config = load_base_config(config_path)
    
    # Try to get encounters from first completed trial
    for trial in data.get("history", []):
        if trial.get("trial_status") == "COMPLETED":
            params = trial.get("parameters", {})
            # Count unique target ship IDs from parameters (e.g., ts1_*, ts2_*, ...)
            target_ids = set()
            for key in params:
                if "_" in key:
                    parts = key.split("_")
                    if parts[0].lower().startswith("ts"):
                        target_ids.add(parts[0].upper())
            
            if target_ids:
                # Infer encounter types (default to head-on for now)
                encounters = []
                for ts_id in sorted(target_ids):
                    encounters.append({
                        "id": ts_id,
                        "encounter_type": "head-on" if len(encounters) == 0 else "crossing-give-way"
                    })
                return encounters
    
    # Fallback: default to two target ships
    return [
        {"id": "TS1", "encounter_type": "head-on"},
        {"id": "TS2", "encounter_type": "crossing-give-way"},
    ]


def show_trials(df):
    """Display all trials in a readable format."""
    print("\n" + "="*100)
    print("Available Trials:")
    print("="*100)
    
    # Select key columns for display
    display_cols = ["trial", "type", "status", "danger_score", "min_dist", 
                    "own_ship_collision", "own_ship_grounding", "own_ship_navigation_failure",
                    "own_ship_reaches_end_waypoint"]
    
    available_cols = [c for c in display_cols if c in df.columns]
    display_df = df[available_cols].copy()
    
    # Format numeric columns
    for col in display_df.columns:
        if display_df[col].dtype in ['float64', 'float32']:
            display_df[col] = display_df[col].apply(lambda x: f"{x:.4f}" if isinstance(x, float) else x)
    
    print(display_df.to_string(index=False))
    print("="*100 + "\n")


def prepare_scenario(parameters, encounters, config_path, encounter_settings_path, os_initial):
    """Build (base config, trafficgen spawn requests) from BO parameters."""
    config = copy.deepcopy(load_base_config(config_path))
    spawn_requests = prepare_spawn_requests_trafficgen(
        parameters, encounters, os_initial, config_path, encounter_settings_path,
    )
    return config, spawn_requests


def replay_trial(trial_index, df, data, paths, encounters, os_initial, save_animation=False):
    """Replay a trial using replay_best_trial."""
    
    if trial_index >= len(df):
        print(f"Trial {trial_index} not found. Available: 0-{len(df)-1}")
        return
    
    # Get trial details
    trial_row = df.iloc[trial_index]
    trial_data = data["history"][trial_index]
    parameters = trial_data["parameters"]
    metrics = trial_data["metrics"]
    
    # Print info
    print(f"\n{'='*70}")
    print(f"Replaying Trial {trial_index} ({trial_data['trial_type']})")
    print(f"{'='*70}")
    print(f"Status: {trial_data['trial_status']}")
    
    # Check if trial failed
    if trial_data['trial_status'] == "FAILED":
        if "error" in metrics:
            print(f"Error: {metrics['error']}")
        print(f"{'='*70}\n")
        return
    
    # Print detailed metrics for successful trials
    print(f"Danger Score: {metrics['danger_score']:.4f}")
    print(f"Min Distance: {metrics['min_dist']:.2f}m")
    print(f"Own Ship Collision: {metrics['own_ship_collision']}")
    print(f"Own Ship Grounding: {metrics['own_ship_grounding']}")
    print(f"Own Ship Nav Failure: {metrics['own_ship_navigation_failure']}")
    print(f"Own Ship Reaches End: {metrics['own_ship_reaches_end_waypoint']}")
    if "tar_ships_collision" in metrics:
        print(f"Target Ships Collision: {metrics['tar_ships_collision']}")
        print(f"Target Ships Grounding: {metrics['tar_ships_grounding']}")
        print(f"Target Ships Nav Failure: {metrics['tar_ships_navigation_failure']}")
    print(f"{'='*70}\n")
    
    anim_path = paths["save_dir"] / f"trial_{trial_index}_animation.mp4" if save_animation else None
    
    target_ship_ids = [enc["id"] for enc in encounters]
    
    # Create scenario preparer with closure over all parameters
    def scenario_preparer(params):
        return prepare_scenario(params, encounters, paths["config_path"], 
                               paths["encounter_settings_path"], os_initial)
    
    # Replay
    replay_best_trial(
        scenario_preparer_fn=scenario_preparer,
        target_ship_ids=target_ship_ids,
        best_parameters=parameters,
        root=ROOT,
        save_dir=paths["save_dir"],
        best_anim_path=anim_path,
    )


def main():
    parser = argparse.ArgumentParser(
        description="Replay BO trials from results",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples (from test_run directory):
  python ../../orchestrator/bo_replay_trial.py --scenario two_target_ship_ho.yaml --trial 5
  python ../../orchestrator/bo_replay_trial.py --scenario single_target_ship_ho.yaml --best

Examples (from root directory):
  python orchestrator/bo_replay_trial.py --scenario test_run/open_sea_two_ts_ho_bo/two_target_ship_ho.yaml --trial 5
        """
    )
    parser.add_argument("--scenario", required=True, help="Scenario config file path (absolute or relative)")
    parser.add_argument("--trial", type=int, default=None, help="Trial index to replay")
    parser.add_argument("--best", action="store_true", help="Replay best trial (lowest danger_score)")
    parser.add_argument("--list", action="store_true", help="List all trials")
    parser.add_argument("--save", action="store_true", help="Save animation to MP4")
    args = parser.parse_args()
    
    # Get current directory (where script is called from)
    current_dir = Path.cwd()
    
    try:
        # Load scenario info - extracts directory from scenario path
        paths = get_scenario_info(args.scenario, current_dir)
        
        # Load results
        df, data = load_csv_and_json(paths)
        if df is None or data is None:
            return
        
        # Parse encounters from results
        encounters = parse_encounters_from_results(data, paths["config_path"], 
                                                  paths["encounter_settings_path"])
        
        # Default own ship initial state
        os_initial = {
            "position": {"north": 0.0, "east": 0.0},
            "sog": 10.0,
            "cog": 0.0,
            "heading": 0.0,
            "navStatus": "Under way using engine",
        }
        
        # Handle --list
        if args.list:
            show_trials(df)
            return
        
        # Handle --best
        if args.best:
            best_idx = df["danger_score"].idxmin()
            args.trial = best_idx
            print(f"\nBest trial found: Trial {best_idx} (danger_score={df.iloc[best_idx]['danger_score']:.4f})")
        
        # Validate trial index
        if args.trial is None:
            parser.print_help()
            return
        
        # Replay
        replay_trial(args.trial, df, data, paths, encounters, os_initial, save_animation=args.save)
    
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main() or 0)
