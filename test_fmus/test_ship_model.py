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
import numpy as np

from libcosimpy.CosimExecution import CosimExecution
from libcosimpy.CosimSlave import CosimLocalSlave
from libcosimpy.CosimManipulator import CosimManipulator
from libcosimpy.CosimObserver import CosimObserver
from libcosimpy.CosimEnums import CosimVariableType
from old_function.utils import GetVariableInfo, GetVariableIndex


def plot_real_ts(observer, slave_index, vr, label, sample_count=200000):
    t_ns, _, y = observer.time_series_real_samples(slave_index, value_reference=vr, sample_count=sample_count, from_step=0)
    t_s = np.array(t_ns) / 1e9
    plt.plot(t_s, y, label=label)

stepSize = int(1e7)     # 0.01 s
stopTime = int(60e9)    # 60 s

execution = CosimExecution.from_step_size(step_size=stepSize)
observer = CosimObserver.create_time_series()
execution.add_observer(observer=observer)

manip = CosimManipulator.create_override()
execution.add_manipulator(manipulator=manip)

fmu_path = str(ROOT / "FMUs" / "ShipModel.fmu")
slave = CosimLocalSlave(fmu_path=fmu_path, instance_name="SHIP_MODEL")
idx = execution.add_local_slave(local_slave=slave)
vars_ = execution.slave_variables(slave_index=idx)

# ---- Set ship parameters (adapt as needed; keep reasonable magnitudes) ----
# Use try/except so script doesn't die if some variable names differ
ship_params = {
    "dead_weight_tonnage": 3850000.0,
    "coefficient_of_deadweight_to_displacement": 0.7,
    "bunkers": 200000.0,
    "ballast": 200000.0,
    "length_of_ship": 80.0,
    "width_of_ship": 16.0,
    "added_mass_coefficient_in_surge": 0.4,
    "added_mass_coefficient_in_sway": 0.4,
    "added_mass_coefficient_in_yaw": 0.4,
    "mass_over_linear_friction_coefficient_in_surge": 130.0,
    "mass_over_linear_friction_coefficient_in_sway": 18.0,
    "mass_over_linear_friction_coefficient_in_yaw": 90.0,
    "nonlinear_friction_coefficient_in_surge": 2400.0,
    "nonlinear_friction_coefficient_in_sway": 4000.0,
    "nonlinear_friction_coefficient_in_yaw": 400.0,
    "rho_seawater": 1025.0,
    "rho_air": 1.2,
    "front_above_water_height": 8.0,
    "side_above_water_height": 8.0,
    "cx": 0.5,
    "cy": 0.7,
    "cn": 0.08,
    "initial_north_position_m": 0.0,
    "initial_east_position_m": 0.0,
    "initial_yaw_angle_rad": float(np.deg2rad(45.0)),
    "initial_forward_speed_m_per_s": 0.0,
    "initial_sideways_speed_m_per_s": 0.0,
    "initial_yaw_rate_rad_per_s": 0.0,
}

for name, val in ship_params.items():
    try:
        vr = GetVariableIndex(vars_, name)
        execution.real_initial_value(idx, vr, float(val))
    except Exception:
        pass

# ---- Observe outputs ----
out_names = ["north", "east", "yaw_angle_rad", "forward_speed", "sideways_speed", "yaw_rate"]
out_vrs = {}
for nm in out_names:
    vr, tp = GetVariableInfo(vars_, nm)
    observer.start_time_series(idx, value_reference=vr, variable_type=tp)
    out_vrs[nm] = vr

# ---- Input VRs ----
inputs = ["thrust_force", "rudder_force_v", "rudder_force_r",
          "wind_speed", "wind_dir_rad", "current_speed", "current_dir_rad"]
in_vrs = {}
for nm in inputs:
    vr, tp = GetVariableInfo(vars_, nm)
    if tp != CosimVariableType.REAL:
        raise RuntimeError(f"Expected REAL input {nm}, got {tp}")
    in_vrs[nm] = vr

def thrust_force(t_s):
    # ramp thrust up, then hold (units depend on your model; start modest)
    if t_s < 10:
        return 2e6 * (t_s / 10.0)  # 0 -> 200kN-ish equivalent (if N)
    return 2e5

def rudder_force_v(t_s): return 0.0
def rudder_force_r(t_s): return 0.0

def wind_speed(t_s): return 0.0
def wind_dir_rad(t_s): return 0.0
def current_speed(t_s): return 0.0
def current_dir_rad(t_s): return 0.0

# Initialize inputs
ts0 = 0.0
manip.slave_real_values(idx, [in_vrs["thrust_force"]],   [thrust_force(ts0)])
manip.slave_real_values(idx, [in_vrs["rudder_force_v"]], [rudder_force_v(ts0)])
manip.slave_real_values(idx, [in_vrs["rudder_force_r"]], [rudder_force_r(ts0)])
manip.slave_real_values(idx, [in_vrs["wind_speed"]],     [wind_speed(ts0)])
manip.slave_real_values(idx, [in_vrs["wind_dir_rad"]],   [wind_dir_rad(ts0)])
manip.slave_real_values(idx, [in_vrs["current_speed"]],  [current_speed(ts0)])
manip.slave_real_values(idx, [in_vrs["current_dir_rad"]],[current_dir_rad(ts0)])

t = 0
while t < stopTime:
    ts = t / 1e9
    manip.slave_real_values(idx, [in_vrs["thrust_force"]],   [thrust_force(ts)])
    manip.slave_real_values(idx, [in_vrs["rudder_force_v"]], [rudder_force_v(ts)])
    manip.slave_real_values(idx, [in_vrs["rudder_force_r"]], [rudder_force_r(ts)])
    manip.slave_real_values(idx, [in_vrs["wind_speed"]],     [wind_speed(ts)])
    manip.slave_real_values(idx, [in_vrs["wind_dir_rad"]],   [wind_dir_rad(ts)])
    manip.slave_real_values(idx, [in_vrs["current_speed"]],  [current_speed(ts)])
    manip.slave_real_values(idx, [in_vrs["current_dir_rad"]],[current_dir_rad(ts)])

    execution.step()
    t += stepSize

# ---- Plot ----
plt.figure()
plot_real_ts(observer, idx, out_vrs["north"], "north")
plot_real_ts(observer, idx, out_vrs["east"], "east")
plt.xlabel("Time [s]")
plt.ylabel("Position [m]")
plt.grid(True)
plt.legend()

plt.figure()
plot_real_ts(observer, idx, out_vrs["forward_speed"], "forward_speed")
plot_real_ts(observer, idx, out_vrs["sideways_speed"], "sideways_speed")
plt.xlabel("Time [s]")
plt.ylabel("Speed [m/s]")
plt.grid(True)
plt.legend()

plt.figure()
plot_real_ts(observer, idx, out_vrs["yaw_angle_rad"], "yaw_angle_rad")
plt.xlabel("Time [s]")
plt.ylabel("Yaw [rad]")
plt.grid(True)
plt.legend()

plt.figure()
plot_real_ts(observer, idx, out_vrs["yaw_rate"], "yaw_rate")
plt.xlabel("Time [s]")
plt.ylabel("Yaw rate [rad/s]")
plt.grid(True)
plt.legend()

plt.show()
