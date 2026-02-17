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

import matplotlib.pyplot as plt
from libcosimpy.CosimExecution import CosimExecution
from libcosimpy.CosimSlave import CosimLocalSlave
from libcosimpy.CosimManipulator import CosimManipulator
from libcosimpy.CosimObserver import CosimObserver
from libcosimpy.CosimEnums import CosimVariableType
from old_function.utils import *

# =========================
# Instantiate CoSimInstance
# =========================
stopTime = 2000 # Number of steps in nano seconds (int)
stepSize = 0.01 # Number of nano seconds (int)

name        = "Ship Path-Following Test"
instance    = CoSimInstance(instanceName= name, stopTime=stopTime, stepSize=stepSize)

# =========================
# Adding slaves
# =========================
# ThrottleController.fmu
throttle_controller_fmu_path = str(ROOT / "FMUs" / "ThrottleController.fmu")
instance.AddSlave(name="THROTTLE_CONTROLLER", 
                  path=throttle_controller_fmu_path)

# MachinerySystem.fmu
machinery_system_fmu_path = str(ROOT / "FMUs" / "MachinerySystem.fmu")
instance.AddSlave(name="MACHINERY_SYSTEM", 
                  path=machinery_system_fmu_path)

# ShipModel.fmu
ship_model_fmu_path = str(ROOT / "FMUs" / "ShipModel.fmu")
instance.AddSlave(name="SHIP_MODEL", 
                  path=ship_model_fmu_path)

# =========================
# Set Initial Values
# =========================
# Throttle Controller
throttle_controller_params = {
    "kp": 0.05,
    "ki": 0.0001
}
instance.SetInitialValues(slaveName="THROTTLE_CONTROLLER", 
                         params=throttle_controller_params)

# Machinery System
machinery_system_params = {
    "mso_mode": 2,
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
instance.SetInitialValues(slaveName="MACHINERY_SYSTEM", 
                         params=machinery_system_params)

# # Ship Model
ship_model_params = {
    "dead_weight_tonnage": 3850000,
    "coefficient_of_deadweight_to_displacement": 0.7,
    "bunkers": 200000,
    "ballast": 200000,
    "length_of_ship": 80,
    "width_of_ship": 16,
    "added_mass_coefficient_in_surge": 0.4,
    "added_mass_coefficient_in_sway": 0.4,
    "added_mass_coefficient_in_yaw": 0.4,
    "mass_over_linear_friction_coefficient_in_surge": 130,
    "mass_over_linear_friction_coefficient_in_sway": 18,
    "mass_over_linear_friction_coefficient_in_yaw": 90,
    "nonlinear_friction_coefficient_in_surge": 2400,
    "nonlinear_friction_coefficient_in_sway": 4000,
    "nonlinear_friction_coefficient_in_yaw": 400,
    "rho_seawater": 1025,
    "rho_air": 1.2,
    "front_above_water_height": 8,
    "side_above_water_height": 8,
    "cx": 0.5,
    "cy": 0.7,
    "cn": 0.08,
    "initial_north_position_m": 0.0,
    "initial_east_position_m": 0.0,
    "initial_yaw_angle_rad": 0.0,
    "initial_forward_speed_m_per_s": 0.0,
    "initial_sideways_speed_m_per_s": 0.0,
    "initial_yaw_rate_rad_per_s": 0.0 
}
instance.SetInitialValues(slaveName="SHIP_MODEL", 
                         params=ship_model_params)

# =========================
# Setup Observer – Outputs
# =========================
# Throttle Controller
instance.AddObserverTimeSeriesWithLabel(name="throttle_cmd", slaveName="THROTTLE_CONTROLLER", variable="throttle_cmd", var_label="Throttle [-]")

# Machinery System
instance.AddObserverTimeSeriesWithLabel(name="thrust_force", slaveName="MACHINERY_SYSTEM", variable="thrust_force", var_label="Force [kN]")
instance.AddObserverTimeSeriesWithLabel(name="shaft_speed_rpm", slaveName="MACHINERY_SYSTEM", variable="shaft_speed_rpm", var_label="Shaft Speed [RPM]")
instance.AddObserverTimeSeriesWithLabel(name="cmd_load_fraction_me", slaveName="MACHINERY_SYSTEM", variable="cmd_load_fraction_me", var_label="Load Fraction [-]")
instance.AddObserverTimeSeriesWithLabel(name="cmd_load_fraction_hsg", slaveName="MACHINERY_SYSTEM", variable="cmd_load_fraction_hsg", var_label="Load Fraction [-]")
instance.AddObserverTimeSeriesWithLabel(name="power_me", slaveName="MACHINERY_SYSTEM", variable="power_me", var_label="Power [kW]")
instance.AddObserverTimeSeriesWithLabel(name="available_power_me", slaveName="MACHINERY_SYSTEM", variable="available_power_me", var_label="Power [kW]")
instance.AddObserverTimeSeriesWithLabel(name="power_electrical", slaveName="MACHINERY_SYSTEM", variable="power_electrical", var_label="Power [kW]")
instance.AddObserverTimeSeriesWithLabel(name="available_power_electrical", slaveName="MACHINERY_SYSTEM", variable="available_power_electrical", var_label="Power [kW]")
instance.AddObserverTimeSeriesWithLabel(name="power", slaveName="MACHINERY_SYSTEM", variable="power", var_label="Power [kW]")
instance.AddObserverTimeSeriesWithLabel(name="propulsion_power", slaveName="MACHINERY_SYSTEM", variable="propulsion_power", var_label="Power k[W]")
instance.AddObserverTimeSeriesWithLabel(name="fuel_rate_me", slaveName="MACHINERY_SYSTEM", variable="fuel_rate_me", var_label="Fuel Rate [kg/s]")
instance.AddObserverTimeSeriesWithLabel(name="fuel_rate_hsg", slaveName="MACHINERY_SYSTEM", variable="fuel_rate_hsg", var_label="Fuel Rate [kg/s]")
instance.AddObserverTimeSeriesWithLabel(name="fuel_rate", slaveName="MACHINERY_SYSTEM", variable="fuel_rate", var_label="Fuel Rate [kg/s]")
instance.AddObserverTimeSeriesWithLabel(name="fuel_consumption_me", slaveName="MACHINERY_SYSTEM", variable="fuel_consumption_me", var_label="Fuel Mass [kg]")
instance.AddObserverTimeSeriesWithLabel(name="fuel_consumption_hsg", slaveName="MACHINERY_SYSTEM", variable="fuel_consumption_hsg", var_label="Fuel Mass [kg]")
instance.AddObserverTimeSeriesWithLabel(name="fuel_consumption", slaveName="MACHINERY_SYSTEM", variable="fuel_consumption", var_label="Fuel Mass [kg]")
instance.AddObserverTimeSeriesWithLabel(name="motor_torque", slaveName="MACHINERY_SYSTEM", variable="motor_torque", var_label="Torque [Nm]")
instance.AddObserverTimeSeriesWithLabel(name="hybrid_shaft_generator_torque", slaveName="MACHINERY_SYSTEM", variable="hybrid_shaft_generator_torque", var_label="Torque [Nm]")

# Ship Model
instance.AddObserverTimeSeriesWithLabel(name="total_ship_speed", slaveName="SHIP_MODEL", variable="total_ship_speed", var_label="Speed [m/s]")

# =========================
# Simulate
# =========================
while instance.time < instance.stopTime:
     # Get values
    desired_shaft_speed_rpm = 450
    
    # if instance.time > instance.stopTime/4:
    #     desired_shaft_speed_rpm = 300
    
    # if instance.time > instance.stopTime/2:
    #     desired_shaft_speed_rpm = 550
    #     # instance.SingleVariableManipulation(slaveName="MACHINERY_SYSTEM", slaveVar="mso_mode", value=0)
    
    # if instance.time > instance.stopTime*3/4:
    #     desired_shaft_speed_rpm = 400
        # instance.SingleVariableManipulation(slaveName="MACHINERY_SYSTEM", slaveVar="mso_mode", value=1)
    
    # Get values
    measured_shaft_speed_rpm    = instance.GetLastValue(slaveName="MACHINERY_SYSTEM", slaveVar="shaft_speed_rpm")
    throttle_cmd                = instance.GetLastValue(slaveName="THROTTLE_CONTROLLER", slaveVar="throttle_cmd")
    thrust_force                = instance.GetLastValue(slaveName="MACHINERY_SYSTEM", slaveVar="thrust_force")
    
    # Set values
    instance.SingleVariableManipulation(slaveName="THROTTLE_CONTROLLER", slaveVar="desired_shaft_speed_rpm", value=desired_shaft_speed_rpm)
    instance.SingleVariableManipulation(slaveName="THROTTLE_CONTROLLER", slaveVar="measured_shaft_speed_rpm", value=measured_shaft_speed_rpm)
    instance.SingleVariableManipulation(slaveName="MACHINERY_SYSTEM", slaveVar="load_perc", value=throttle_cmd)
    instance.SingleVariableManipulation(slaveName="SHIP_MODEL", slaveVar="thrust_force", value=thrust_force)
    
    
    # Step
    instance.execution.step()
    instance.time += instance.stepSize
    
# =========================
# Plot
# =========================
key_group_list = [["throttle_cmd"],
                  ["thrust_force"],
                  ["shaft_speed_rpm"],
                  ["cmd_load_fraction_me", "cmd_load_fraction_hsg"],
                  ["power_me", "available_power_me"],
                  ["power_electrical", "available_power_electrical"],
                  ["power", "propulsion_power"],
                  ["fuel_rate_me", "fuel_rate_hsg", "fuel_rate"],
                  ["fuel_consumption_me", "fuel_consumption_hsg", "fuel_consumption"],
                  ["motor_torque", "hybrid_shaft_generator_torque"],
                  ["total_ship_speed"]]

instance.JoinPlotTimeSeries(key_group_list,  create_title= False, legend= True, show_instance_name=False)