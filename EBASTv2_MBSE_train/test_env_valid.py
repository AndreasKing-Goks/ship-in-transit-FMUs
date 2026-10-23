from gymnasium.utils.env_checker import check_env
from pathlib import Path
import sys
import os

# Ensure libcosim DLL is found
dll_dir = Path(sys.prefix) / "Lib" / "site-packages" / "libcosimpy" / "libcosimc"
os.add_dll_directory(str(dll_dir))

## PATH HELPER (OBLIGATORY)
# project root = two levels up from this file
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from EBASTv2_MBSE_train.env_EBASTv2_MBSE import EBASTv2EnvMBSE
from EBASTv2_MBSE_train.MBSE_episode_logger import log_episode_recap
from orchestrator.scenario_config import generate_spawn_request_bank, load_spawn_requests_bank_path

from itertools import product
import numpy as np

# =========================
# Load the Configuration
# =========================
# Get the config path
config_path = ROOT / "EBASTv2_MBSE_train" / "EBASTv2_MBSE.yaml",

# Get the save path for animation
save_path = ROOT / "EBASTv2_MBSE_train" /  "saved_animation" / "EBASTv2MBSE_test.mp4"

# Log path
log_path = ROOT / "EBASTv2_MBSE_train" / "logs" / "EBASTv2MBSE_episode_recap.txt"

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
target_start_times  = [0.0, 150,   300.0]

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
# NOTE: 
# BOUND SHOULD NOT BE ABSOLUTE POSITION (-1000, 11000)
# BUT RATHER RELATIVE (-11000, 11000)
margin  = 1000
rounded_north_bound = [-(max_dist+margin), (max_dist+margin)]
rounded_east_bound  = [-(max_dist+margin), (max_dist+margin)]

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
    detailed_reward=True,
    )

# Check the env validity
try_check_env = True
if try_check_env:
    try:
        check_env(env)
        print("Environment passes all chekcs!")
    except Exception as e:
        print(f"Environment has issues: {e}")
        
        print("Start debugging observation")
        obs, info = env.reset()

        for key, space in env.observation_space.spaces.items():
            value = obs[key]
            ok = space.contains(value)

            print("\n", key)
            print("contains:", ok)
            print("shape:", value.shape, "expected:", space.shape)
            print("dtype:", value.dtype, "expected:", space.dtype)
            print("min/max:", np.min(value), np.max(value))
            print("value:", value)
        
        print("ABORT TRAINING")
        sys.exit(1)  # non-zero exit code stops the script

def check_observation(env, obs, step):
    for key, space in env.observation_space.spaces.items():

        value = np.asarray(obs[key])

        if not space.contains(value):
            print(f"\n[ERROR] Step {step}")
            print(f"Observation: {key}")
            print(f"Value: {value}")
            print(f"Shape: {value.shape}")
            print(f"Dtype: {value.dtype}")
            print(f"Expected min: {space.low}")
            print(f"Expected max: {space.high}")

            return False

    return True

try:
    for episode in range(10):

        obs, info = env.reset()

        if not check_observation(env, obs, 0):
            raise ValueError("Invalid reset observation")

        for step in range(1000):

            action = env.action_space.sample()

            obs, reward, terminated, truncated, info = env.step(action)

            if not check_observation(env, obs, step + 1):
                raise ValueError("Invalid step observation")

            if terminated or truncated:
                break

        print(f"Episode {episode + 1} passed")

finally:
    env.close()