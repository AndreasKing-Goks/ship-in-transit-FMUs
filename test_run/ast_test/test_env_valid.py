from pathlib import Path
import sys
import os

# Workaround for OpenMP duplicate runtime
os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

# Important to keep to preven sb3-contrib importing torch from ARS that causes error
import torch
print("Torch:", torch.__version__)

from sb3_contrib import RecurrentPPO
from stable_baselines3.common.evaluation import evaluate_policy
from gymnasium.utils.env_checker import check_env

# Ensure libcosim DLL is found
dll_dir = Path(sys.prefix) / "Lib" / "site-packages" / "libcosimpy" / "libcosimc"
os.add_dll_directory(str(dll_dir))

## PATH HELPER (OBLIGATORY)
# project root = two levels up from this file
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from RL_env.env import EBASTv2Env
from RL_env.episode_logger import log_episode_recap
from orchestrator.scenario_config import generate_spawn_request_bank, load_spawn_requests_bank_path

import numpy as np

# =========================
# Load the Configuration
# =========================
# Get the config path
config_path = ROOT / "test_run" / "ast_test" / "test_ast.yaml"

# Get the save path for animation
save_path = ROOT / "saved_animation" / "test_env_valid.gif"

# Get the encounter settings path
encounter_settings_path = ROOT / "test_run" / "ast_test" / "encounter_settings.json"

# Log path
log_path = ROOT / "test_run" / "ast_test" / "logs" / "episode_recap.txt"

# Spawn requests bank path
spawn_requests_bank_path = ROOT / "test_run" / "ast_test" / "spawn_request_bank.pkl"

# =========================
# Instantiate the environment wrapper
# =========================
# Generates/collect spawn requests
spawn_requests_bank_path    = generate_spawn_request_bank(config_path=config_path,
                                                          encounter_settings_path=encounter_settings_path,
                                                          spawn_requests_bank_path=spawn_requests_bank_path,
                                                          n_cases=100,
                                                          training_case_ratio=0.8,                              # Specifically for RL-env
                                                          overwrite=False)
spawn_requests_bank         = load_spawn_requests_bank_path(spawn_requests_bank_path)

# Instantiate the RL-environment wrapper class
env = EBASTv2Env(
    ROOT=ROOT,
    config_path=config_path,
    encounter_settings_path=encounter_settings_path,
    spawn_requests_bank=spawn_requests_bank
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