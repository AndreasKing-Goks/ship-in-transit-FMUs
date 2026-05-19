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

from RL_env.env import EBASTv2Env
from RL_env.episode_logger import log_episode_recap
from orchestrator.scenario_config import generate_spawn_request_bank, load_spawn_requests_bank_path

# =========================
# Load the Configuration
# =========================
# Get the config path
config_path = ROOT / "test_run" / "ast_test" / "test_ast.yaml"

# Get the save path for animation
save_path = ROOT / "saved_animation" / "test_ast_2.gif"

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
                                                          n_cases=50,
                                                          overwrite=False)
spawn_requests_bank         = load_spawn_requests_bank_path(spawn_requests_bank_path)

# Instantiate the RL-environment wrapper class
env = EBASTv2Env(
    ROOT=ROOT,
    config_path=config_path,
    encounter_settings_path=encounter_settings_path,
    spawn_requests_bank=spawn_requests_bank
    )

obs, info   = env.reset(seed=46)

action_list = [[30,2500],
               [15,2000],
               [0,2000],
               [0,2000],
               [-15,2000]]

i = 0
while i < 5:
    # action = env.action_space.sample()
    action  = env._normalize_action(action_list[i])
    env.step(action)
    i += 1

# =========================
# Log the episode
# =========================
log_episode_recap(env=env, log_path=log_path)
print(f"Episode recap saved to: {log_path}")

# =========================
# Animation and Plot
# =========================
# Available formats:
# - .mp4
# - .gif
# - .avi
# - .mov

# Animate Simulation
env.instance.AnimateFleetTrajectory(
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
env.instance.PlotFleetTrajectory(mode="quick", ship_scale=1.0)