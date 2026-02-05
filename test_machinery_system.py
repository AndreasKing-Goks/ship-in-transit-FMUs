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

name        = "Ship Path-Following Test"
instance    = CoSimInstance(instanceName= name, stopTime=stopTime, stepSize=stepSize)

# =========================
# Adding slaves
# =========================
# MachinerySystem.fmu
machinery_system_fmu_path = str(ROOT / "FMUs" / "MachinerySystem.fmu")
instance.AddSlave(name="MACHINERY_SYSTEM", 
                  path=machinery_system_fmu_path)

# =========================
# Set Initial Values
# =========================
# Machinery System
machinery_system_params = {
    "hotel_load": 200000,
    "rated_speed_main_engine_rpm": 1000,
    "linear_friction_main_engine": 68,
    "linear_friction_hybrid_shaft_generator": 57,
    "gear_ratio_between_main_engine_and_propeller": 0.6,
    "gear_ratio_between_hybrid_shaft_generator_and_propeller": 0.6,
    "propeller_inertia": 6000,
    "propeller_speed_to_torque_coefficient": 7.5,
    "propeller_diameter": 3.1,
    "propeller_speed_to_thrust_force_coefficient": 1.7,
    "specific_fuel_consumption_coefficients_me_a_coeff": 128.89,
    "specific_fuel_consumption_coefficients_me_b_coeff": -168.93,
    "specific_fuel_consumption_coefficients_me_c_coeff": 246.76,
    "specific_fuel_consumption_coefficients_dg_a_coeff": 180.71,
    "specific_fuel_consumption_coefficients_dg_b_coeff": -289.90,
    "specific_fuel_consumption_coefficients_dg_c_coeff": 324.90,
    "omega": 0.0,
    "d_omega": 0.0,
    "main_engine_capacity_spec": 2160e3,
    "diesel_gen_capacity_spec": 510e3,
}
instance.SetInitalValues(slaveName="MACHINERY_SYSTEM", 
                         params=machinery_system_params)

# =========================
# Setup Observer – Outputs
# =========================
instance.AddObserverTimeSeriesWithLabel(name="thrust_force", slaveName="MACHINERY_SYSTEM", variable="thrust_force", var_lable="Force [kN]")
instance.AddObserverTimeSeriesWithLabel(name="shaft_speed_rpm", slaveName="MACHINERY_SYSTEM", variable="shaft_speed_rpm", var_lable="Shaft Speed [RPM]")
instance.AddObserverTimeSeriesWithLabel(name="cmd_load_fraction_me", slaveName="MACHINERY_SYSTEM", variable="cmd_load_fraction_me", var_lable="Load Fraction [-]")
instance.AddObserverTimeSeriesWithLabel(name="cmd_load_fraction_hsg", slaveName="MACHINERY_SYSTEM", variable="cmd_load_fraction_hsg", var_lable="Load Fraction [-]")
instance.AddObserverTimeSeriesWithLabel(name="power_me", slaveName="MACHINERY_SYSTEM", variable="power_me", var_lable="Power [kW]")
instance.AddObserverTimeSeriesWithLabel(name="available_power_me", slaveName="MACHINERY_SYSTEM", variable="available_power_me", var_lable="Power [kW]")
instance.AddObserverTimeSeriesWithLabel(name="power_electrical", slaveName="MACHINERY_SYSTEM", variable="power_electrical", var_lable="Power [kW]")
instance.AddObserverTimeSeriesWithLabel(name="available_power_electrical", slaveName="MACHINERY_SYSTEM", variable="available_power_electrical", var_lable="Power [kW]")
instance.AddObserverTimeSeriesWithLabel(name="power", slaveName="MACHINERY_SYSTEM", variable="power", var_lable="Power [kW]")
instance.AddObserverTimeSeriesWithLabel(name="propulsion_power", slaveName="MACHINERY_SYSTEM", variable="propulsion_power", var_lable="Power k[W]")
instance.AddObserverTimeSeriesWithLabel(name="fuel_rate_me", slaveName="MACHINERY_SYSTEM", variable="fuel_rate_me", var_lable="Fuel Rate [kg/s]")
instance.AddObserverTimeSeriesWithLabel(name="fuel_rate_hsg", slaveName="MACHINERY_SYSTEM", variable="fuel_rate_hsg", var_lable="Fuel Rate [kg/s]")
instance.AddObserverTimeSeriesWithLabel(name="fuel_rate", slaveName="MACHINERY_SYSTEM", variable="fuel_rate", var_lable="Fuel Rate [kg/s]")
instance.AddObserverTimeSeriesWithLabel(name="fuel_consumption_me", slaveName="MACHINERY_SYSTEM", variable="fuel_consumption_me", var_lable="Fuel Mass [kg]")
instance.AddObserverTimeSeriesWithLabel(name="fuel_consumption_hsg", slaveName="MACHINERY_SYSTEM", variable="fuel_consumption_hsg", var_lable="Fuel Mass [kg]")
instance.AddObserverTimeSeriesWithLabel(name="fuel_consumption", slaveName="MACHINERY_SYSTEM", variable="fuel_consumption", var_lable="Fuel Mass [kg]")
instance.AddObserverTimeSeriesWithLabel(name="motor_torque", slaveName="MACHINERY_SYSTEM", variable="motor_torque", var_lable="Torque [Nm]")
instance.AddObserverTimeSeriesWithLabel(name="hybrid_shaft_generator_torque", slaveName="MACHINERY_SYSTEM", variable="hybrid_shaft_generator_torque", var_lable="Torque [Nm]")

# =========================
# Input Metadata
# =========================
engine_index        = instance.slaves_index["MACHINERY_SYSTEM"]
engine_variables    = instance.slaves_variables["MACHINERY_SYSTEM"]
load_perc_vr        = GetVariableIndex(engine_variables, 'load_perc')
mso_mode_vr         = GetVariableIndex(engine_variables, 'mso_mode')

# =========================
# Simulate
# =========================
while instance.time < instance.stopTime:
     # Get values
    load_perc = 0.5
    mso_mode = 2
    
    # if time > stopTime/4:
    #     load_perc=0.5
    
    if instance.time > instance.stopTime/2:
        mso_mode = 1
    
    if instance.time > instance.stopTime*3/4:
        mso_mode = 0
    
    # Set values
    instance.manipulator.slave_real_values(engine_index, [load_perc_vr], [load_perc])
    instance.manipulator.slave_integer_values(engine_index, [mso_mode_vr], [mso_mode])
    
    # Step
    instance.execution.step()
    instance.time += instance.stepSize
    
# =========================
# Plot
# =========================
key_group_list = [["thrust_force"],
                  ["shaft_speed_rpm"],
                  ["cmd_load_fraction_me", "cmd_load_fraction_hsg"],
                  ["power_me", "available_power_me"],
                  ["power_electrical", "available_power_electrical"],
                  ["power", "propulsion_power"],
                  ["fuel_rate_me", "fuel_rate_hsg", "fuel_rate"],
                  ["fuel_consumption_me", "fuel_consumption_hsg", "fuel_consumption"],
                  ["motor_torque", "hybrid_shaft_generator_torque"]]

instance.JoinPlotTimeSeries(key_group_list,  create_title= False, legend= True, show_instance_name=False)