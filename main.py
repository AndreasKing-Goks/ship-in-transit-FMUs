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

# =========================
# Load the Configuration
# =========================
import yaml

# Get the path
config_path = ROOT / "simu_config.yaml"

with config_path.open("r", encoding="utf-8") as f:
    config = yaml.safe_load(f)
    
simu_config     = config["simulation"]
ship_configs    = config["ships"]
    
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
# Set up the FMUs for all ship assets
instance.add_ship(ship_configs=ship_configs, ROOT=ROOT)

# =========================
# Simulate
# =========================
instance.Simulate()

# # =========================
# # Plot
# # =========================
instance.PlotFleetTrajectory()

key_group_list = [
    ["OS0.new_throttle_cmd"],
    ["OS0.new_rudder_angle_deg"],
    ["OS0.beta_own_to_tar_1"],
    ["OS0.tcpa_own_to_tar_1"],
    ["OS0.dcpa_own_to_tar_1"],
    ["OS0.dist_own_to_tar_1"],
    ["OS0.rr_own_to_tar_1"],
]

# Plot Time Series
instance.JoinPlotTimeSeries(list(reversed(key_group_list)),  create_title= False, legend= True, show_instance_name=False, show=True)