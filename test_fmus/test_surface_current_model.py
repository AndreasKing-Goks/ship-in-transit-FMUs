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

from cosim_instance import CoSimInstance

# =========================
# Instantiate CoSimInstance
# =========================
stopTime = 1000 # Number of steps in nano seconds (int)
stepSize = 0.01 # Number of nano seconds (int)

name        = "Surface Current Test"
instance    = CoSimInstance(instanceName= name, stopTime=stopTime, stepSize=stepSize)

# =========================
# Adding slaves
# =========================
# SurfaceCurrentModel.fmu
surface_current_model_fmu_path = str(ROOT / "FMUs" / "SurfaceCurrentModel.fmu")
instance.AddSlave(name="SURFACE_CURRENT", 
                  path=surface_current_model_fmu_path)

# =========================
# Set Initial Values
# =========================
# Surface Current
surface_current_params = {
    "seed": 0,
    "initial_current_speed": 0.01,
    "current_speed_decay_rate": 0.5,
    "current_speed_standard_deviation": 0.015,
    "initial_current_direction_deg": 0.0,
    "current_direction_deg_decay_rate": 0.5,
    "current_direction_deg_standard_deviation": 5.0,
    "clip_speed_nonnegative": True,
    "fail_outputs_zero": True
}
instance.SetInitialValues(slaveName="SURFACE_CURRENT", 
                         params=surface_current_params)

# =========================
# Setup Observer – Outputs
# =========================
instance.AddObserverTimeSeriesWithLabel(name="current_speed", slaveName="SURFACE_CURRENT", variable="current_speed", var_label="Speed [m/s]")
instance.AddObserverTimeSeriesWithLabel(name="current_direction_deg", slaveName="SURFACE_CURRENT", variable="current_direction_deg", var_label="Angle [deg]")
instance.AddObserverTimeSeriesWithLabel(name="mean_current_speed", slaveName="SURFACE_CURRENT", variable="mean_current_speed", var_label="Speed [m/s]")
instance.AddObserverTimeSeriesWithLabel(name="mean_current_direction_deg", slaveName="SURFACE_CURRENT", variable="mean_current_direction_deg", var_label="Angle [deg]")

# =========================
# Simulate
# =========================
while instance.time < instance.stopTime:
     # Get values
    mean_current_speed = 0.25
    mean_current_direction_deg = 90.0
    
    if instance.time > instance.stopTime/4:
        mean_current_speed=0.75
        mean_current_direction_deg = 135.0
    
    if instance.time > instance.stopTime/2:
        mean_current_speed = 1.25
        mean_current_direction_deg = 45.0
    
    if instance.time > instance.stopTime*3/4:
        mean_current_speed = 0.25
        mean_current_direction_deg = 0.0
    
    # Set values
    instance.SingleVariableManipulation(slaveName   ="SURFACE_CURRENT", 
                                        slaveVar    ="mean_current_speed", 
                                        value       =mean_current_speed)
    instance.SingleVariableManipulation(slaveName   ="SURFACE_CURRENT", 
                                        slaveVar    ="mean_current_direction_deg", 
                                        value       =mean_current_direction_deg)
    
    # Step
    instance.execution.step()
    instance.time += instance.stepSize
    
# =========================
# Plot
# =========================
key_group_list = [
    ["current_speed", "mean_current_speed"],
    ["current_direction_deg", "mean_current_direction_deg"]
]

instance.JoinPlotTimeSeries(key_group_list=list(reversed(key_group_list)), create_title=False, legend=True, show_instance_name=False)