"""
Replay a specific BO trial from CSV results with animation and plots.

Usage:
    python replay_trial.py --trial 5                    # Replay trial 5
    python replay_trial.py --trial 5 --save             # Replay and save MP4
    python replay_trial.py --best                       # Replay best trial (lowest danger_score)
    python replay_trial.py --list                       # Show all trials
"""

from pathlib import Path
import sys
import json
import argparse
import os
import pandas as pd

# Ensure libcosim DLL is found
dll_dir = Path(sys.prefix) / "Lib" / "site-packages" / "libcosimpy" / "libcosimc"
os.add_dll_directory(str(dll_dir))

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from orchestrator.bo_core import replay_best_trial
from orchestrator.scenario_config import prepare_trial_config_direct, get_target_ship_ids, load_base_config

CONFIG_PATH = Path(__file__).with_name("single_target_ship_ho.yaml")
RESULTS_PATH = Path(__file__).with_name("ax_results.json")
CSV_PATH = Path(__file__).with_name("ax_results_metrics.csv")
SAVE_DIR = ROOT / "saved_animation"


def load_csv_and_json():
    """Load both CSV and JSON results."""
    if not CSV_PATH.exists():
        print(f"CSV file not found: {CSV_PATH}")
        print("Run main.py first to generate results.")
        return None, None
    
    if not RESULTS_PATH.exists():
        print(f"JSON file not found: {RESULTS_PATH}")
        return None, None
    
    df = pd.read_csv(CSV_PATH)
    with open(RESULTS_PATH, "r") as f:
        data = json.load(f)
    
    return df, data


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


def get_trial_details(df, trial_index):
    """Get detailed info for a trial."""
    if trial_index >= len(df):
        return None
    
    row = df.iloc[trial_index]
    return row


def replay_trial(trial_index, df, data, save_animation=False):
    """Replay a trial using replay_best_trial."""
    
    if trial_index >= len(df):
        print(f"Trial {trial_index} not found. Available: 0-{len(df)-1}")
        return
    
    # Get trial details
    trial_row = df.iloc[trial_index]
    trial_data = data["history"][trial_index]
    parameters = dict(trial_data["parameters"])  # Make a copy
    
    # Inject missing wp_count for any target ship (when max_intermediate_wps=0)
    base_config = load_base_config(CONFIG_PATH)
    target_ship_ids = get_target_ship_ids(base_config)
    for ts_id in target_ship_ids:
        prefix = ts_id.lower()
        if f"{prefix}_wp_count" not in parameters:
            parameters[f"{prefix}_wp_count"] = 0
    
    metrics = trial_data["metrics"]
    
    # Print info
    print(f"\n{'='*70}")
    print(f"Replaying Trial {trial_index} ({trial_data['trial_type']})")
    print(f"{'='*70}")
    print(f"Status: {trial_data['trial_status']}")
    print(f"Danger Score: {metrics['danger_score']:.4f}")
    print(f"Min Distance: {metrics['min_dist']:.2f}m")
    print(f"Own Ship Collision: {metrics['own_ship_collision']}")
    print(f"Own Ship Grounding: {metrics['own_ship_grounding']}")
    print(f"Own Ship Nav Failure: {metrics['own_ship_navigation_failure']}")
    print(f"Own Ship Reaches End: {metrics['own_ship_reaches_end_waypoint']}")
    print(f"Target Ships Collision: {metrics['tar_ships_collision']}")
    print(f"Target Ships Grounding: {metrics['tar_ships_grounding']}")
    print(f"Target Ships Nav Failure: {metrics['tar_ships_navigation_failure']}")
    print(f"{'='*70}\n")
    
    anim_path = SAVE_DIR / f"trial_{trial_index}_animation.mp4" if save_animation else None
    
    # Replay
    replay_best_trial(
        scenario_preparer_fn=lambda p: (
            prepare_trial_config_direct(p, CONFIG_PATH, target_ship_ids),
            None,
        ),
        target_ship_ids=target_ship_ids,
        best_parameters=parameters,
        root=ROOT,
        save_dir=SAVE_DIR,
        best_anim_path=anim_path,
    )


def main():
    parser = argparse.ArgumentParser(
        description="Replay BO trials from CSV results",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python replay_trial.py --trial 5              # Replay trial 5
  python replay_trial.py --trial 5 --save       # Replay and save MP4
  python replay_trial.py --best                 # Replay best trial (lowest danger_score)
  python replay_trial.py --list                 # Show all trials
        """
    )
    parser.add_argument("--trial", type=int, default=None, help="Trial index to replay")
    parser.add_argument("--best", action="store_true", help="Replay best trial (lowest danger_score)")
    parser.add_argument("--list", action="store_true", help="List all trials")
    parser.add_argument("--save", action="store_true", help="Save animation to MP4")
    args = parser.parse_args()
    
    # Load results
    df, data = load_csv_and_json()
    if df is None or data is None:
        return
    
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
        print("\n❌ Please specify --trial <index>, --best, or --list")
        return
    
    # Replay trial
    replay_trial(args.trial, df, data, save_animation=args.save)


if __name__ == "__main__":
    main()
