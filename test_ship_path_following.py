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
stopTime = 1e11 # Number of steps in nano seconds (int)
stepSize = 1e7 # Number of nano seconds (int)

name        = "Ship Path-Following Test"
instance    = CoSimInstance(instanceName= name, stopTime=stopTime, stepSize=stepSize)

# =========================
# Adding slaves
# =========================
# Autopilot.fmu
autopilot_fmu_path = str(ROOT / "FMUs" / "Autopilot.fmu")
instance.AddSlave(name="AUTOPILOT", 
                  path=autopilot_fmu_path)

# ShaftSpeedController.fmu
shaft_speed_controller_fmu_path = str(ROOT / "FMUs" / "ShaftSpeedController.fmu")
instance.AddSlave(name="SHAFT_SPEED_CONTROLLER", 
                  path=shaft_speed_controller_fmu_path)

# ThrottleController.fmu
throttle_controller_fmu_path = str(ROOT / "FMUs" / "ThrottleController.fmu")
instance.AddSlave(name="THROTTLE_CONTROLLER", 
                  path=throttle_controller_fmu_path)

# MachinerySystem.fmu
machinery_system_fmu_path = str(ROOT / "FMUs" / "MachinerySystem.fmu")
instance.AddSlave(name="MACHINERY_SYSTEM", 
                  path=machinery_system_fmu_path)

# Rudder.fmu
rudder_fmu_path = str(ROOT / "FMUs" / "Rudder.fmu")
instance.AddSlave(name="RUDDER", 
                  path=rudder_fmu_path)

# ShipModel.fmu
ship_model_fmu_path = str(ROOT / "FMUs" / "ShipModel.fmu")
instance.AddSlave(name="SHIP_MODEL", 
                  path=ship_model_fmu_path)

# SurfaceCurrentModel.fmu
surface_current_model_fmu_path = str(ROOT / "FMUs" / "SurfaceCurrentModel.fmu")
instance.AddSlave(name="SURFACE_CURRENT", 
                  path=surface_current_model_fmu_path)

# WindModel.fmu
wind_model_fmu_path = str(ROOT / "FMUs" / "WindModel.fmu")
instance.AddSlave(name="WIND", 
                  path=wind_model_fmu_path)

# SetPointsManager.fmu
setpoints_manager_fmu_path = str(ROOT / "FMUs" / "SetPointsManager.fmu")
instance.AddSlave(name="SET_POINTS_MANAGER", 
                  path=setpoints_manager_fmu_path)

# =========================
# Set Initial Values
# =========================
# Autopilot
autopilot_params = {
    "ra": 300,
    "r": 1000,
    "ki_ct": 0.002,
    "integrator_limit": 4000,
    "kp": 1.5,
    "ki": 75,
    "kd": 0.005,
    "max_rudder_rate_deg_per_sec": 2.3,
    "max_rudder_angle_deg": 35
}
instance.SetInitalValues(slaveName="AUTOPILOT", 
                         params=autopilot_params)

# Shaft Speed Controller
shaft_speed_controller_params = {
    "kp": 2.50,
    "ki": 0.025
}
instance.SetInitalValues(slaveName="SHAFT_SPEED_CONTROLLER", 
                         params=shaft_speed_controller_params)

# Throttle Controller
throttle_controller_params = {
    "kp": 0.05,
    "ki": 0.0001
}
instance.SetInitialValues(slaveName="THROTTLE_CONTROLLER", 
                         params=throttle_controller_params)

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
    "mso_mode": 0 ,
}
instance.SetInitialValues(slaveName="MACHINERY_SYSTEM", 
                         params=machinery_system_params)

# Rudder
rudder_params = {
    "rudder_angle_to_sway_force_coefficient": 50e3,
    "rudder_angle_to_yaw_force_coefficient": 500e3,
    "max_rudder_angle_negative_deg": np.deg2rad(-35),
    "max_rudder_angle_positive_deg": np.deg2rad(35)
}
instance.SetInitialValues(slaveName="RUDDER", 
                         params=rudder_params)

# Ship Model
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
    "initial_yaw_angle_rad": np.deg2rad(45),
    "initial_forward_speed_m_per_s": 0.0,
    "initial_sideways_speed_m_per_s": 0.0,
    "initial_yaw_rate_rad_per_s": 0.0 
}
instance.SetInitialValues(slaveName="SHIP_MODEL", 
                         params=ship_model_params)

# # Surface Current
# surface_current_params = {
#     "seed": 0,
#     "initial_current_speed": 0.01,
#     "current_velocity_decay_rate": 0.0075,
#     "current_velocity_standard_deviation": 0.025,
#     "initial_current_direction": np.deg2rad(0.0),
#     "current_direction_decay_rate": 0.0,
#     "current_direction_standard_deviation": 0.0,
#     "clip_speed_nonnegative": True
# }
# instance.SetInitialValues(slaveName="SURFACE_CURRENT", 
#                          params=surface_current_params)

# # Wind
# wind_params ={
#     "initial_mean_wind_speed": 5.0,
#     "mean_wind_speed_decay_rate": 0.025,
#     "mean_wind_speed_standard_deviation": 0.005,
#     "initial_wind_direction": np.deg2rad(0.0),
#     "wind_direction_decay_rate": 0.025,
#     "wind_direction_standard_deviation": 0.025,
#     "minimum_mean_wind_speed": 0.0,
#     "maximum_mean_wind_speed": 42.0,
#     "minimum_wind_gust_frequency": 0.06,
#     "maximum_wind_gust_frequency": 0.4,
#     "wind_gust_frequency_discrete_unit_count": 100,
#     "wind_evaluation_height": 5.0,
#     "U10": 10.0,
#     "kappa_parameter": 0.0026,
#     "clip_speed_nonnegative": True,
# }
# instance.SetInitialValues(slaveName="WIND", 
#                          params=wind_params)

# Set Points Manager
set_points_manager_params = {
    "ra": 300,
    "max_inter_wp": 3,
    "wp_start_north": 0.0,
    "wp_start_east": 0.0,
    "wp_start_speed": 5.0,
    "wp_1_north": 2500.0,
    "wp_1_east": 2500.0,
    "wp_1_speed": 5.0,
    "wp_2_north": 5000.0,
    "wp_2_east": 3750.0,
    "wp_2_speed": 5.0,
    "wp_3_north": 5000.0,
    "wp_3_east": 8750.0,
    "wp_3_speed": 5.0,
    "wp_end_north": 10000.0,
    "wp_end_east": 10000.0,
    "wp_end_speed": 5.0,
}
instance.SetInitialValues(slaveName="SET_POINTS_MANAGER", 
                         params=set_points_manager_params)

# # =========================
# # Add Model Connections
# # =========================
# # Input to Autopilot
# instance.AddSlaveConnection(slaveInputName="AUTOPILOT", slaveInputVar="north", 
#                             slaveOutputName="SHIP_MODEL", slaveOutputVar="north")
# instance.AddSlaveConnection(slaveInputName="AUTOPILOT", slaveInputVar="east", 
#                             slaveOutputName="SHIP_MODEL", slaveOutputVar="east")
# instance.AddSlaveConnection(slaveInputName="AUTOPILOT", slaveInputVar="heading_mea", 
#                             slaveOutputName="SHIP_MODEL", slaveOutputVar="yaw_angle")
# instance.AddSlaveConnection(slaveInputName="AUTOPILOT", slaveInputVar="prev_wp_north", 
#                             slaveOutputName="SET_POINTS_MANAGER", slaveOutputVar="prev_wp_north")
# instance.AddSlaveConnection(slaveInputName="AUTOPILOT", slaveInputVar="prev_wp_east", 
#                             slaveOutputName="SET_POINTS_MANAGER", slaveOutputVar="prev_wp_east")
# instance.AddSlaveConnection(slaveInputName="AUTOPILOT", slaveInputVar="next_wp_north", 
#                             slaveOutputName="SET_POINTS_MANAGER", slaveOutputVar="next_wp_north")
# instance.AddSlaveConnection(slaveInputName="AUTOPILOT", slaveInputVar="next_wp_east", 
#                             slaveOutputName="SET_POINTS_MANAGER", slaveOutputVar="next_wp_east")

# # Input to Shaft Speed Controller
# instance.AddSlaveConnection(slaveInputName="SHAFT_SPEED_CONTROLLER", slaveInputVar="desired_ship_speed", 
#                             slaveOutputName="SET_POINTS_MANAGER", slaveOutputVar="prev_wp_speed")
# instance.AddSlaveConnection(slaveInputName="SHAFT_SPEED_CONTROLLER", slaveInputVar="measured_ship_speed", 
#                             slaveOutputName="SHIP_MODEL", slaveOutputVar="measured_speed")

# # Input to Throttle Controller
# instance.AddSlaveConnection(slaveInputName="THROTTLE_CONTROLLER", slaveInputVar="desired_shaft_speed", 
#                             slaveOutputName="SHAFT_SPEED_CONTROLLER", slaveOutputVar="shaft_speed_cmd")
# instance.AddSlaveConnection(slaveInputName="THROTTLE_CONTROLLER", slaveInputVar="measured_shaft_speed", 
#                             slaveOutputName="MACHINERY_SYSTEM", slaveOutputVar="shaft_speed_rpm")

# # Input to Machinery System
# instance.AddSlaveConnection(slaveInputName="MACHINERY_SYSTEM", slaveInputVar="load_perc",
#                             slaveOutputName="THROTTLE_CONTROLLER", slaveOutputVar="throttle_cmd")

# # Input to Rudder
# instance.AddSlaveConnection(slaveInputName="RUDDER", slaveInputVar="rudder_angle_deg",
#                             slaveOutputName="AUTOPILOT", slaveOutputVar="rudder_angle")
# instance.AddSlaveConnection(slaveInputName="RUDDER", slaveInputVar="yaw_angle_rad",
#                             slaveOutputName="SHIP_MODEL", slaveOutputVar="yaw_angle")
# instance.AddSlaveConnection(slaveInputName="RUDDER", slaveInputVar="forward_speed",
#                             slaveOutputName="SHIP_MODEL", slaveOutputVar="forward_speed")

# # Input to Ship Model
# instance.AddSlaveConnection(slaveInputName="SHIP_MODEL", slaveInputVar="thrust_force",
#                             slaveOutputName="MACHINERY_SYSTEM", slaveOutputVar="thrust_force")
# instance.AddSlaveConnection(slaveInputName="SHIP_MODEL", slaveInputVar="rudder_force_v",
#                             slaveOutputName="RUDDER", slaveOutputVar="rudder_force_v")
# instance.AddSlaveConnection(slaveInputName="SHIP_MODEL", slaveInputVar="rudder_force_r",
#                             slaveOutputName="RUDDER", slaveOutputVar="rudder_force_r")

# # Input to Set Points Manager
# instance.AddSlaveConnection(slaveInputName="SET_POINTS_MANAGER", slaveInputVar="north", 
#                             slaveOutputName="SHIP_MODEL", slaveOutputVar="north")
# instance.AddSlaveConnection(slaveInputName="SET_POINTS_MANAGER", slaveInputVar="east", 
#                             slaveOutputName="SHIP_MODEL", slaveOutputVar="east")

# # =========================
# # Simulate
# # =========================
# instance.Simulate()