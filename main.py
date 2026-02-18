from pathlib import Path
import sys
import os

# Ensure libcosim DLL is found
dll_dir = Path(sys.prefix) / "Lib" / "site-packages" / "libcosimpy" / "libcosimc"
os.add_dll_directory(str(dll_dir))

## PATH HELPER (OBLIGATORY)
# project root = two levels up from this file
ROOT = Path(__file__).resolve().parents[0]
sys.path.insert(0, str(ROOT))

from sit_cosim import ShipInTransitCoSimulation
from utils import compile_ship_params

# =========================
# Load the Configuration
# =========================

import yaml

# Get the path
config_path = ROOT / "simu_config.yaml"

with config_path.open("r", encoding="utf-8") as f:
    config = yaml.safe_load(f)
    
simu_config     = config["simulation"]
ships_config    = config["ships"]

ship_count      = len(ships_config)
    
# =========================
# Instantiate the Scheduler
# =========================

# Name
instanceName    = simu_config["instanceName"]

# Time
stopTime        = simu_config["stopTime"] # Number of steps in seconds (int)
stepSize        = simu_config["stepSize"] # Number of seconds (int)

# Instantiate
instance = ShipInTransitCoSimulation(instanceName=instanceName, stopTime=stopTime, stepSize=stepSize)

# =========================
# Build the Ships
# =========================
# NOTE:
# Be sure to add target ship first, then add own ship to properly setup the Colav system

# # Add target ship(s). NOTE: Target ship starts from entry 2 onwards
for ts_config in ships_config[1:]:
    ts_param = compile_ship_params(ts_config)
    
# Add own ship. NOTE: Own ship always in located in the first entry
os_param = compile_ship_params(ships_config[0])

print(ships_config[0]['SHIP_CONNECTIONS'])