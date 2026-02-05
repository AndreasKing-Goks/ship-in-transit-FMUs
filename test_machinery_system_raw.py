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

# PARAMETERS
stopTime = 1e11 # Number of steps in nano seconds (int)
stepSize = 1e7 # Number of nano seconds (int)

# Execution setup
execution = CosimExecution.from_step_size(step_size=stepSize)

# Observer
observe_time_series = CosimObserver.create_time_series()
execution.add_observer(observer=observe_time_series)
observe_last = CosimObserver.create_last_value()
execution.add_observer(observer=observe_last)

# Manipulator
manipulator = CosimManipulator.create_override()
execution.add_manipulator(manipulator=manipulator)

# Adding slaves
engine_fmu_path = str(ROOT / "FMUs" / "MachinerySystem.fmu")
engine_slave = CosimLocalSlave(fmu_path=engine_fmu_path, instance_name='ENGINE')
engine_index = execution.add_local_slave(local_slave=engine_slave)
engine_variables = execution.slave_variables(slave_index=engine_index)

# =========================
# Set Initial Values
# =========================

# =========================
# Machinery System Configuration
# =========================
mso_mode_val = 0
mso_mode_vr = GetVariableIndex(engine_variables, 'mso_mode')
execution.integer_initial_value(
    slave_index=engine_index,
    variable_reference=mso_mode_vr,
    value=mso_mode_val
)

hotel_load_val = 200000
hotel_load_vr = GetVariableIndex(engine_variables, 'hotel_load')
execution.real_initial_value(
    slave_index=engine_index,
    variable_reference=hotel_load_vr,
    value=hotel_load_val
)

rated_speed_main_engine_rpm_val = 1000
rated_speed_main_engine_rpm_vr = GetVariableIndex(
    engine_variables, 'rated_speed_main_engine_rpm'
)
execution.real_initial_value(
    slave_index=engine_index,
    variable_reference=rated_speed_main_engine_rpm_vr,
    value=rated_speed_main_engine_rpm_val
)

linear_friction_main_engine_val = 68
linear_friction_main_engine_vr = GetVariableIndex(
    engine_variables, 'linear_friction_main_engine'
)
execution.real_initial_value(
    slave_index=engine_index,
    variable_reference=linear_friction_main_engine_vr,
    value=linear_friction_main_engine_val
)

linear_friction_hybrid_shaft_generator_val = 57
linear_friction_hybrid_shaft_generator_vr = GetVariableIndex(
    engine_variables, 'linear_friction_hybrid_shaft_generator'
)
execution.real_initial_value(
    slave_index=engine_index,
    variable_reference=linear_friction_hybrid_shaft_generator_vr,
    value=linear_friction_hybrid_shaft_generator_val
)

gear_ratio_between_main_engine_and_propeller_val = 0.6
gear_ratio_between_main_engine_and_propeller_vr = GetVariableIndex(
    engine_variables, 'gear_ratio_between_main_engine_and_propeller'
)
execution.real_initial_value(
    slave_index=engine_index,
    variable_reference=gear_ratio_between_main_engine_and_propeller_vr,
    value=gear_ratio_between_main_engine_and_propeller_val
)

gear_ratio_between_hybrid_shaft_generator_and_propeller_val = 0.6
gear_ratio_between_hybrid_shaft_generator_and_propeller_vr = GetVariableIndex(
    engine_variables, 'gear_ratio_between_hybrid_shaft_generator_and_propeller'
)
execution.real_initial_value(
    slave_index=engine_index,
    variable_reference=gear_ratio_between_hybrid_shaft_generator_and_propeller_vr,
    value=gear_ratio_between_hybrid_shaft_generator_and_propeller_val
)

propeller_inertia_val = 6000
propeller_inertia_vr = GetVariableIndex(engine_variables, 'propeller_inertia')
execution.real_initial_value(
    slave_index=engine_index,
    variable_reference=propeller_inertia_vr,
    value=propeller_inertia_val
)

propeller_speed_to_torque_coefficient_val = 7.5
propeller_speed_to_torque_coefficient_vr = GetVariableIndex(
    engine_variables, 'propeller_speed_to_torque_coefficient'
)
execution.real_initial_value(
    slave_index=engine_index,
    variable_reference=propeller_speed_to_torque_coefficient_vr,
    value=propeller_speed_to_torque_coefficient_val
)

propeller_diameter_val = 3.1
propeller_diameter_vr = GetVariableIndex(engine_variables, 'propeller_diameter')
execution.real_initial_value(
    slave_index=engine_index,
    variable_reference=propeller_diameter_vr,
    value=propeller_diameter_val
)

propeller_speed_to_thrust_force_coefficient_val = 1.7
propeller_speed_to_thrust_force_coefficient_vr = GetVariableIndex(
    engine_variables, 'propeller_speed_to_thrust_force_coefficient'
)
execution.real_initial_value(
    slave_index=engine_index,
    variable_reference=propeller_speed_to_thrust_force_coefficient_vr,
    value=propeller_speed_to_thrust_force_coefficient_val
)

# =========================
# Specific Fuel Consumption
# =========================

specific_fuel_consumption_coefficients_me_a_coeff_val = 128.89
specific_fuel_consumption_coefficients_me_a_coeff_vr = GetVariableIndex(
    engine_variables, 'specific_fuel_consumption_coefficients_me_a_coeff'
)
execution.real_initial_value(
    slave_index=engine_index,
    variable_reference=specific_fuel_consumption_coefficients_me_a_coeff_vr,
    value=specific_fuel_consumption_coefficients_me_a_coeff_val
)

specific_fuel_consumption_coefficients_me_b_coeff_val = -168.93
specific_fuel_consumption_coefficients_me_b_coeff_vr = GetVariableIndex(
    engine_variables, 'specific_fuel_consumption_coefficients_me_b_coeff'
)
execution.real_initial_value(
    slave_index=engine_index,
    variable_reference=specific_fuel_consumption_coefficients_me_b_coeff_vr,
    value=specific_fuel_consumption_coefficients_me_b_coeff_val
)

specific_fuel_consumption_coefficients_me_c_coeff_val = 246.76
specific_fuel_consumption_coefficients_me_c_coeff_vr = GetVariableIndex(
    engine_variables, 'specific_fuel_consumption_coefficients_me_c_coeff'
)
execution.real_initial_value(
    slave_index=engine_index,
    variable_reference=specific_fuel_consumption_coefficients_me_c_coeff_vr,
    value=specific_fuel_consumption_coefficients_me_c_coeff_val
)

specific_fuel_consumption_coefficients_dg_a_coeff_val = 180.71
specific_fuel_consumption_coefficients_dg_a_coeff_vr = GetVariableIndex(
    engine_variables, 'specific_fuel_consumption_coefficients_dg_a_coeff'
)
execution.real_initial_value(
    slave_index=engine_index,
    variable_reference=specific_fuel_consumption_coefficients_dg_a_coeff_vr,
    value=specific_fuel_consumption_coefficients_dg_a_coeff_val
)

specific_fuel_consumption_coefficients_dg_b_coeff_val = -289.90
specific_fuel_consumption_coefficients_dg_b_coeff_vr = GetVariableIndex(
    engine_variables, 'specific_fuel_consumption_coefficients_dg_b_coeff'
)
execution.real_initial_value(
    slave_index=engine_index,
    variable_reference=specific_fuel_consumption_coefficients_dg_b_coeff_vr,
    value=specific_fuel_consumption_coefficients_dg_b_coeff_val
)

specific_fuel_consumption_coefficients_dg_c_coeff_val = 324.90
specific_fuel_consumption_coefficients_dg_c_coeff_vr = GetVariableIndex(
    engine_variables, 'specific_fuel_consumption_coefficients_dg_c_coeff'
)
execution.real_initial_value(
    slave_index=engine_index,
    variable_reference=specific_fuel_consumption_coefficients_dg_c_coeff_vr,
    value=specific_fuel_consumption_coefficients_dg_c_coeff_val
)

# =========================
# Tunable Parameters
# =========================

omega_val = 0.0
omega_vr = GetVariableIndex(engine_variables, 'omega')
execution.real_initial_value(
    slave_index=engine_index,
    variable_reference=omega_vr,
    value=omega_val
)

d_omega_val = 0.0
d_omega_vr = GetVariableIndex(engine_variables, 'd_omega')
execution.real_initial_value(
    slave_index=engine_index,
    variable_reference=d_omega_vr,
    value=d_omega_val
)

# =========================
# Engine Capacity Parameters
# =========================

main_engine_capacity_spec_val = 2160e3
main_engine_capacity_spec_vr = GetVariableIndex(
    engine_variables, 'main_engine_capacity_spec'
)
execution.real_initial_value(
    slave_index=engine_index,
    variable_reference=main_engine_capacity_spec_vr,
    value=main_engine_capacity_spec_val
)

diesel_gen_capacity_spec_val = 510e3
diesel_gen_capacity_spec_vr = GetVariableIndex(
    engine_variables, 'diesel_gen_capacity_spec'
)
execution.real_initial_value(
    slave_index=engine_index,
    variable_reference=diesel_gen_capacity_spec_vr,
    value=diesel_gen_capacity_spec_val
)


# =========================
# Setup Observer – Outputs
# =========================

# thrust 
thrust_force_vr, thrust_force_type = GetVariableInfo(engine_variables, 'thrust_force')
observe_time_series.start_time_series(
    engine_index, value_reference=thrust_force_vr, variable_type=thrust_force_type
)

# shaft_speed_rpm
shaft_speed_rpm_vr, shaft_speed_rpm_type = GetVariableInfo(engine_variables, 'shaft_speed_rpm')
observe_time_series.start_time_series(
    engine_index, value_reference=shaft_speed_rpm_vr, variable_type=shaft_speed_rpm_type
)

# cmd_load_fraction_me
cmd_load_fraction_me_vr, cmd_load_fraction_me_type = GetVariableInfo(
    engine_variables, 'cmd_load_fraction_me'
)
observe_time_series.start_time_series(
    engine_index, value_reference=cmd_load_fraction_me_vr, variable_type=cmd_load_fraction_me_type
)

# cmd_load_fraction_hsg
cmd_load_fraction_hsg_vr, cmd_load_fraction_hsg_type = GetVariableInfo(
    engine_variables, 'cmd_load_fraction_hsg'
)
observe_time_series.start_time_series(
    engine_index, value_reference=cmd_load_fraction_hsg_vr, variable_type=cmd_load_fraction_hsg_type
)

# power_me
power_me_vr, power_me_type = GetVariableInfo(engine_variables, 'power_me')
observe_time_series.start_time_series(
    engine_index, value_reference=power_me_vr, variable_type=power_me_type
)

# available_power_me
available_power_me_vr, available_power_me_type = GetVariableInfo(
    engine_variables, 'available_power_me'
)
observe_time_series.start_time_series(
    engine_index, value_reference=available_power_me_vr, variable_type=available_power_me_type
)

# power_electrical
power_electrical_vr, power_electrical_type = GetVariableInfo(
    engine_variables, 'power_electrical'
)
observe_time_series.start_time_series(
    engine_index, value_reference=power_electrical_vr, variable_type=power_electrical_type
)

# available_power_electrical
available_power_electrical_vr, available_power_electrical_type = GetVariableInfo(
    engine_variables, 'available_power_electrical'
)
observe_time_series.start_time_series(
    engine_index,
    value_reference=available_power_electrical_vr,
    variable_type=available_power_electrical_type
)

# power
power_vr, power_type = GetVariableInfo(engine_variables, 'power')
observe_time_series.start_time_series(
    engine_index, value_reference=power_vr, variable_type=power_type
)

# propulsion_power
propulsion_power_vr, propulsion_power_type = GetVariableInfo(
    engine_variables, 'propulsion_power'
)
observe_time_series.start_time_series(
    engine_index, value_reference=propulsion_power_vr, variable_type=propulsion_power_type
)

# fuel_rate_me
fuel_rate_me_vr, fuel_rate_me_type = GetVariableInfo(engine_variables, 'fuel_rate_me')
observe_time_series.start_time_series(
    engine_index, value_reference=fuel_rate_me_vr, variable_type=fuel_rate_me_type
)

# fuel_rate_hsg
fuel_rate_hsg_vr, fuel_rate_hsg_type = GetVariableInfo(
    engine_variables, 'fuel_rate_hsg'
)
observe_time_series.start_time_series(
    engine_index, value_reference=fuel_rate_hsg_vr, variable_type=fuel_rate_hsg_type
)

# fuel_rate
fuel_rate_vr, fuel_rate_type = GetVariableInfo(engine_variables, 'fuel_rate')
observe_time_series.start_time_series(
    engine_index, value_reference=fuel_rate_vr, variable_type=fuel_rate_type
)

# fuel_consumption_me
fuel_consumption_me_vr, fuel_consumption_me_type = GetVariableInfo(
    engine_variables, 'fuel_consumption_me'
)
observe_time_series.start_time_series(
    engine_index,
    value_reference=fuel_consumption_me_vr,
    variable_type=fuel_consumption_me_type
)

# fuel_consumption_hsg
fuel_consumption_hsg_vr, fuel_consumption_hsg_type = GetVariableInfo(
    engine_variables, 'fuel_consumption_hsg'
)
observe_time_series.start_time_series(
    engine_index,
    value_reference=fuel_consumption_hsg_vr,
    variable_type=fuel_consumption_hsg_type
)

# fuel_consumption
fuel_consumption_vr, fuel_consumption_type = GetVariableInfo(
    engine_variables, 'fuel_consumption'
)
observe_time_series.start_time_series(
    engine_index,
    value_reference=fuel_consumption_vr,
    variable_type=fuel_consumption_type
)

# motor_torque
motor_torque_vr, motor_torque_type = GetVariableInfo(
    engine_variables, 'motor_torque'
)
observe_time_series.start_time_series(
    engine_index, value_reference=motor_torque_vr, variable_type=motor_torque_type
)

# hybrid_shaft_generator_torque
hybrid_shaft_generator_torque_vr, hybrid_shaft_generator_torque_type = GetVariableInfo(
    engine_variables, 'hybrid_shaft_generator_torque'
)
observe_time_series.start_time_series(
    engine_index,
    value_reference=hybrid_shaft_generator_torque_vr,
    variable_type=hybrid_shaft_generator_torque_type
)


# Input Metadata
load_perc_vr = GetVariableIndex(engine_variables, 'load_perc')

time = 0
while time < stopTime:
     # Get values
    load_perc = 0.25
    
    if time > stopTime/4:
        load_perc=0.5
    
    if time > stopTime/2:
        load_perc = 1
    
    if time > stopTime*3/4:
        load_perc = 0.75
    
    # Set values
    manipulator.slave_real_values(engine_index, [load_perc_vr], [load_perc])
    
    # Step
    execution.step()
    time += stepSize
    
    
# =========================
# Helper for time axis
# =========================
def plot_real_ts(vr, label):
    t, _, y = observe_time_series.time_series_real_samples(
        engine_index,
        value_reference=vr,
        sample_count=100000,
        from_step=0
    )
    plt.plot([x / 1e9 for x in t], y, label=label)


# =========================
# 1. thrust_force
# =========================
plt.figure()
plot_real_ts(thrust_force_vr, "thrust_force")
plt.xlabel("Time [s]")
plt.ylabel("Force")
plt.legend()
plt.grid(True)


# =========================
# 2. shaft_speed_rpm
# =========================
plt.figure()
plot_real_ts(shaft_speed_rpm_vr, "shaft_speed_rpm")
plt.xlabel("Time [s]")
plt.ylabel("Shaft Speed [RPM]")
plt.legend()
plt.grid(True)


# =========================
# 3. cmd_load_fraction_me + cmd_load_fraction_hsg
# =========================
plt.figure()
plot_real_ts(cmd_load_fraction_me_vr, "cmd_load_fraction_me")
plot_real_ts(cmd_load_fraction_hsg_vr, "cmd_load_fraction_hsg")
plt.xlabel("Time [s]")
plt.ylabel("Load Fraction [-]")
plt.legend()
plt.grid(True)


# =========================
# 4. power_me + available_power_me
# =========================
plt.figure()
plot_real_ts(power_me_vr, "power_me")
plot_real_ts(available_power_me_vr, "available_power_me")
plt.xlabel("Time [s]")
plt.ylabel("Power [W]")
plt.legend()
plt.grid(True)


# =========================
# 5. power_electrical + available_power_electrical
# =========================
plt.figure()
plot_real_ts(power_electrical_vr, "power_electrical")
plot_real_ts(available_power_electrical_vr, "available_power_electrical")
plt.xlabel("Time [s]")
plt.ylabel("Electrical Power [W]")
plt.legend()
plt.grid(True)


# =========================
# 6. power + propulsion_power
# =========================
plt.figure()
plot_real_ts(power_vr, "power")
plot_real_ts(propulsion_power_vr, "propulsion_power")
plt.xlabel("Time [s]")
plt.ylabel("Power [W]")
plt.legend()
plt.grid(True)


# =========================
# 7. fuel_rate_me + fuel_rate_hsg + fuel_rate
# =========================
plt.figure()
plot_real_ts(fuel_rate_me_vr, "fuel_rate_me")
plot_real_ts(fuel_rate_hsg_vr, "fuel_rate_hsg")
plot_real_ts(fuel_rate_vr, "fuel_rate")
plt.xlabel("Time [s]")
plt.ylabel("Fuel Rate")
plt.legend()
plt.grid(True)


# =========================
# 8. fuel_consumption_me + fuel_consumption_hsg + fuel_consumption
# =========================
plt.figure()
plot_real_ts(fuel_consumption_me_vr, "fuel_consumption_me")
plot_real_ts(fuel_consumption_hsg_vr, "fuel_consumption_hsg")
plot_real_ts(fuel_consumption_vr, "fuel_consumption")
plt.xlabel("Time [s]")
plt.ylabel("Fuel Consumption")
plt.legend()
plt.grid(True)


# =========================
# 9. motor_torque + hybrid_shaft_generator_torque
# =========================
plt.figure()
plot_real_ts(motor_torque_vr, "motor_torque")
plot_real_ts(hybrid_shaft_generator_torque_vr, "hybrid_shaft_generator_torque")
plt.xlabel("Time [s]")
plt.ylabel("Torque [Nm]")
plt.legend()
plt.grid(True)

plt.show()