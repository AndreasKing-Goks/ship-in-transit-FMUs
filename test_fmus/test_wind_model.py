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

name        = "Wind Model Test"
instance    = CoSimInstance(instanceName= name, stopTime=stopTime, stepSize=stepSize)

# =========================
# Adding slaves
# =========================
# WindModel.fmu
wind_model_fmu_path = str(ROOT / "FMUs" / "WindModel.fmu")
instance.AddSlave(name="WIND", 
                  path=wind_model_fmu_path)

# =========================
# Set Initial Values
# =========================
# Wind
wind_params ={
    "seed": 0,
    # "initial_mean_wind_speed": 5.0,
    "mean_wind_speed_decay_rate": 0.5,
    "mean_wind_speed_standard_deviation": 1.5,
    "initial_wind_direction_deg": 0.0,
    "wind_direction_deg_decay_rate": 0.5,
    "wind_direction_deg_standard_deviation": 5.0,
    "minimum_mean_wind_speed": 0.0,
    "maximum_mean_wind_speed": 42.0,
    "minimum_wind_gust_frequency": 0.06,
    "maximum_wind_gust_frequency": 0.4,
    "wind_gust_frequency_discrete_unit_count": 100,
    "wind_evaluation_height": 5.0,
    "U10": 10.0,
    "kappa_parameter": 0.0026,
    "clip_speed_nonnegative": True,
    "manual_mean_wind_speed": False,
    "fail_outputs_zero": True
}
instance.SetInitialValues(slaveName="WIND", 
                         params=wind_params)

# =========================
# Setup Observer – Outputs
# =========================
instance.AddObserverTimeSeriesWithLabel(name="wind_speed", slaveName="WIND", variable="wind_speed", var_label="Speed [m/s]")
instance.AddObserverTimeSeriesWithLabel(name="wind_direction_deg", slaveName="WIND", variable="wind_direction_deg", var_label="Angle [deg]")
instance.AddObserverTimeSeriesWithLabel(name="mean_wind_speed", slaveName="WIND", variable="mean_wind_speed", var_label="Speed [m/s]")
instance.AddObserverTimeSeriesWithLabel(name="mean_wind_direction_deg", slaveName="WIND", variable="mean_wind_direction_deg", var_label="Angle [deg]")

# =========================
# Simulate
# =========================
while instance.time < instance.stopTime:
     # Get values
    mean_wind_speed = 12.5
    mean_wind_direction_deg = 90.0
    
    if instance.time > instance.stopTime/4:
        mean_wind_speed=35.0
        mean_wind_direction_deg = 135.0
    
    if instance.time > instance.stopTime/2:
        mean_wind_speed = 5.0
        mean_wind_direction_deg = 45.0
    
    if instance.time > instance.stopTime*3/4:
        mean_wind_speed = 25.0
        mean_wind_direction_deg = 0.0
    
    # Set values
    instance.SingleVariableManipulation(slaveName   ="WIND", 
                                        slaveVar    ="mean_wind_speed", 
                                        value       =mean_wind_speed)
    instance.SingleVariableManipulation(slaveName   ="WIND", 
                                        slaveVar    ="mean_wind_direction_deg", 
                                        value       =mean_wind_direction_deg)
    
    # Step
    instance.execution.step()
    instance.time += instance.stepSize
    
# =========================
# Plot
# =========================
key_group_list = [
    ["wind_speed", "mean_wind_speed"],
    ["wind_direction_deg", "mean_wind_direction_deg"]
]

instance.JoinPlotTimeSeries(key_group_list=list(reversed(key_group_list)), create_title=False, legend=True, show_instance_name=False)