import numpy as np
import pandas as pd
from pathlib import Path
import json
from itertools import product

from pathlib import Path
import sys
import os

# Workaround for OpenMP duplicate runtime
os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

# Important to keep to preven sb3-contrib importing torch from ARS that causes error
import torch
print("Torch:", torch.__version__)

from sb3_contrib import RecurrentPPO

# Ensure libcosim DLL is found
dll_dir = Path(sys.prefix) / "Lib" / "site-packages" / "libcosimpy" / "libcosimc"
os.add_dll_directory(str(dll_dir))

## PATH HELPER (OBLIGATORY)
# project root = two levels up from this file
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from EBASTv2_MBSE_train.env_EBASTv2_MBSE import EBASTv2EnvMBSE
from EBASTv2_MBSE_train.MBSE_episode_logger import log_episode_recap, log_training_args
from EBASTv2_train.evaluate_failure_cases import evaluate_failure_cases
from EBASTv2_core.path_utils import get_RL_model_path


# ========================================
# Experiment configuration
# ========================================

# =========================
# Handle paths
# =========================
# Trained Model Name
model_name                      = "EB-ASTv2_MBSE_2026-10-23_21-11-07_b58a" # Use old env
model_name                      = "EB-ASTv2_MBSE_2026-11-21_09-14-38_e8e7"

# Get the config path
config_path = [
        ROOT / "EBASTv2_MBSE_train" / "EBASTv2_MBSE_improved.yaml",
    ]

# Get the trained model
model_path                      = ROOT / "EBASTv2_MBSE_train" / "trained_model" / model_name / "model" / "model.zip" # Use old env
model_path                      = ROOT / "EBASTv2_MBSE_train" / "trained_model" / model_name / "checkpoints" / "EB-ASTv2_MBSE_2000000_steps.zip"

# Evaluation recap
recap_path                      = ROOT / "EBASTv2_MBSE_train" / "simulated_trained_model" / "evaluation_recap_EBASTv2_MBSE.txt"

# =========================
# Instantiate the environment wrapper
# =========================
## Define custom spawn requests bank
# Maximum distance of the frame
max_dist = 10_000

# Common own ship configuration
own_ship = {
    "start_time": 0.0,
    "north_route": [0, max_dist],
    "east_route": [0, max_dist],
    "speed_setpoints": 5.0,
}

# Target ship routes for each case [(north_routes, east_routes)]
target_routes = [
    ([0, max_dist], [max_dist, 0]),  # Case 0: OS0 Give Way
    ([max_dist, 0], [0, max_dist]),  # Case 1: OS0 Stand On
]
target_start_times  = [0.0, 150, 300.0]

# Generate cases
cases = [
    {
        "case_id": i,
        "spawn_requests": {
            "OS0": own_ship,
            "TS1": {
                "start_time": start_time,
                "north_route": north,
                "east_route": east,
                "speed_setpoints": 5.0,
            },
        },
        "own_ship_initial": None,
        "encounters": None,
    }
    for i, ((north, east), start_time) in enumerate(product(target_routes, target_start_times))
]

# Spawn request bank
margin  = 1000
rounded_north_bound = [-margin, max_dist+margin]
rounded_east_bound  = [-margin, max_dist+margin]

spawn_requests_bank = {
    "path": None,
    "n_cases": len(cases),
    "start_eval_case_id": len(cases),
    "cases": cases,
    "north_bound": rounded_north_bound,
    "east_bound": rounded_east_bound,
    "rounded_north_bound": rounded_north_bound,
    "rounded_east_bound": rounded_east_bound,
}

# Instantiate the RL-environment wrapper class
env = EBASTv2EnvMBSE(
    ROOT=ROOT,
    config_path=config_path,
    spawn_requests_bank=spawn_requests_bank,
    use_fmpy=True
    )

# =========================
# Load the trained model
# =========================
# Load the trained model
recurrent_ppo_model = RecurrentPPO.load(model_path)

CASE_IDX = 3
CONFIG_IDX = 0
FAULT_START = 1700

# Engineering test durations.
# Replace with your approved test specification.
FAULT_DURATIONS = [
    0,
    30,
    60,
    90,
    120,
]

N_TRIALS = 10
BASE_SEED = 42

RESULT_DIR = (
    ROOT
    / "EBASTv2_MBSE_train"
    / "sensor_fault_assessment"
)

RESULT_DIR.mkdir(parents=True, exist_ok=True)


# ========================================
# Single simulation
# ========================================

def run_simulation(
    env,
    model,
    fault_duration,
    seed,
):
    custom_fault = None

    if fault_duration > 0:

        custom_fault = {
            "ship_id": "OS0",
            "sensor": "RADAR",
            "fault": {
                "var": "measurement_valid",
                "val": False,
            },
            "fault_start": FAULT_START,
            "fault_duration": fault_duration,
        }

    obs, reset_info = env.reset(
        seed=seed,
        specific_case_idx=CASE_IDX,
        specific_config_idx=CONFIG_IDX,
        custom_fault=custom_fault,
    )

    lstm_states = None
    episode_starts = np.ones(
        (1,), dtype=bool
    )

    total_reward = 0.0
    step_count = 0

    # print("hi")
    
    while True:

        action, lstm_states = model.predict(
            obs,
            state=lstm_states,
            episode_start=episode_starts,
            deterministic=True,
        )

        obs, reward, terminated, truncated, info = (
            env.step(action)
        )

        total_reward += float(reward)
        step_count += 1

        done = bool(terminated) or bool(truncated)

        episode_starts = np.array(
            [done],
            dtype=bool,
        )
        
        # print(done)
        
        if done:
            break

    return {
        "case_idx": CASE_IDX,
        "config_idx": CONFIG_IDX,
        "seed": seed,
        "fault_start": FAULT_START,
        "fault_duration": fault_duration,
        "total_reward": total_reward,
        "step_count": step_count,
        "terminated": bool(terminated),
        "truncated": bool(truncated),
        "final_info": json.dumps(
            info,
            default=str,
        ),
    }


# ========================================
# Run experiment
# ========================================

results = []

for duration in FAULT_DURATIONS:

    for trial in range(N_TRIALS):

        # seed = BASE_SEED + trial
        seed = BASE_SEED 

        print(
            f"Duration: {duration}s | "
            f"Trial: {trial + 1}/{N_TRIALS} | "
            f"Seed: {seed}"
        )
        
        result = run_simulation(
            env=env,
            model=recurrent_ppo_model,
            fault_duration=duration,
            seed=seed,
        )

        results.append(result)

        # Save incrementally.
        pd.DataFrame(results).to_csv(
            RESULT_DIR / "experiment_results.csv",
            index=False,
        )


# ========================================
# Descriptive statistics
# ========================================

df = pd.DataFrame(results)

summary = df.groupby("fault_duration").agg(
    number_of_trials=("seed", "count"),
    mean_reward=("total_reward", "mean"),
    std_reward=("total_reward", "std"),
    mean_steps=("step_count", "mean"),
    truncated_runs=("truncated", "sum"),
)

summary.to_csv(
    RESULT_DIR / "experiment_summary.csv"
)

print(summary)