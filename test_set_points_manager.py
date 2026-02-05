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

import matplotlib.pyplot as plt
from libcosimpy.CosimExecution import CosimExecution
from libcosimpy.CosimSlave import CosimLocalSlave
from libcosimpy.CosimManipulator import CosimManipulator
from libcosimpy.CosimObserver import CosimObserver
from libcosimpy.CosimEnums import CosimVariableType
from utils import *

# =========================
# Instantiate CoSimInstance
# =========================
stopTime = 100 # Number of steps in nano seconds (int)
stepSize = 0.01 # Number of nano seconds (int)

name        = "Set Points Manager Test"
instance    = CoSimInstance(instanceName= name, stopTime=stopTime, stepSize=stepSize)

# =========================
# Adding slaves
# =========================
# SetPointsManager.fmu
setpoints_manager_fmu_path = str(ROOT / "FMUs" / "SetPointsManager.fmu")
instance.AddSlave(name="SET_POINTS_MANAGER", 
                  path=setpoints_manager_fmu_path)

# =========================
# Set Initial Values
# =========================
# Set Points Manager
set_points_manager_params = {
    "ra": 300,
    "max_inter_wp": 3,
    "wp_start_north": 0.0,
    "wp_start_east": 0.0,
    "wp_start_speed": 5.0,
    "wp_1_north": 2500.0,
    "wp_1_east": 2500.0,
    "wp_1_speed": 6.0,
    "wp_2_north": 5000.0,
    "wp_2_east": 3750.0,
    "wp_2_speed": 7.0,
    "wp_3_north": 5000.0,
    "wp_3_east": 8750.0,
    "wp_3_speed": 8.0,
    "wp_end_north": 10000.0,
    "wp_end_east": 10000.0,
    "wp_end_speed": 9.0,
}
instance.SetInitalValues(slaveName="SET_POINTS_MANAGER", 
                         params=set_points_manager_params)

# =========================
# Setup Observer – Outputs
# =========================
instance.AddObserverTimeSeriesWithLabel(name="prev_wp_north", slaveName="SET_POINTS_MANAGER", variable="prev_wp_north", var_lable="Waypoint [-]")
instance.AddObserverTimeSeriesWithLabel(name="prev_wp_east", slaveName="SET_POINTS_MANAGER", variable="prev_wp_east", var_lable="Waypoint [-]")
instance.AddObserverTimeSeriesWithLabel(name="prev_wp_speed", slaveName="SET_POINTS_MANAGER", variable="prev_wp_speed", var_lable="Speed [m/s]")
instance.AddObserverTimeSeriesWithLabel(name="next_wp_north", slaveName="SET_POINTS_MANAGER", variable="next_wp_north", var_lable="Waypoint [-]")
instance.AddObserverTimeSeriesWithLabel(name="next_wp_east", slaveName="SET_POINTS_MANAGER", variable="next_wp_east", var_lable="Waypoint [-]")
instance.AddObserverTimeSeriesWithLabel(name="next_wp_speed", slaveName="SET_POINTS_MANAGER", variable="next_wp_speed", var_lable="Speed [m/s]")

# =========================
# Input Metadata
# =========================
spm_index       = instance.slaves_index["SET_POINTS_MANAGER"]
spm_variables   = instance.slaves_variables["SET_POINTS_MANAGER"]
north_vr        = GetVariableIndex(spm_variables, 'north')
east_vr         = GetVariableIndex(spm_variables, 'east')

# =========================
# Simulate
# =========================
# Timeseries test data
north_list  = [0, 700, 1500, 2500, 3000, 4000, 5000, 5000, 5000, 5000, 7000, 8500, 10000]
east_list   = [0, 700, 1500, 2500, 3000, 3750, 3750, 6000, 7000, 8750, 8750, 9000, 10000] 
               #              #                 #                 #                  #

for step in range(len(north_list)):
    north = north_list[step]
    east  = east_list[step]
    
    instance.manipulator.slave_real_values(spm_index, [north_vr], [north])
    instance.manipulator.slave_real_values(spm_index, [east_vr], [east])
    
    instance.execution.step()

# =========================
# Plot
# =========================
instance.PlotTimeSeries(separate_plots=True)