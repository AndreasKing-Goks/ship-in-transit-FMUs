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
stopTime = int(20e9)    # 20 s

execution = CosimExecution.from_step_size(step_size=stepSize)
observer = CosimObserver.create_time_series()
execution.add_observer(observer=observer)

manip = CosimManipulator.create_override()
execution.add_manipulator(manipulator=manip)

fmu_path = str(ROOT / "FMUs" / "Rudder.fmu")
slave = CosimLocalSlave(fmu_path=fmu_path, instance_name="RUDDER")
idx = execution.add_local_slave(local_slave=slave)
vars_ = execution.slave_variables(slave_index=idx)

# Optional: set rudder parameters if they exist
# NOTE: Be careful with deg vs rad naming. If variable name includes "_deg", pass degrees.
try:
    execution.real_initial_value(idx, GetVariableIndex(vars_, "rudder_angle_to_sway_force_coefficient"), 50e3)
    execution.real_initial_value(idx, GetVariableIndex(vars_, "rudder_angle_to_yaw_force_coefficient"), 500e3)

    # If your FMU expects degrees limits, pass degrees.
    # If it expects radians, rename variables accordingly in FMU.
    if "max_rudder_angle_negative_deg" in [v.name for v in vars_]:
        execution.real_initial_value(idx, GetVariableIndex(vars_, "max_rudder_angle_negative_deg"), -35.0)
        execution.real_initial_value(idx, GetVariableIndex(vars_, "max_rudder_angle_positive_deg"),  35.0)
except Exception:
    pass

# Outputs
v_vr, v_type = GetVariableInfo(vars_, "rudder_force_v")
r_vr, r_type = GetVariableInfo(vars_, "rudder_force_r")
observer.start_time_series(idx, value_reference=v_vr, variable_type=v_type)
observer.start_time_series(idx, value_reference=r_vr, variable_type=r_type)

# Inputs
rud_vr, rud_type = GetVariableInfo(vars_, "rudder_angle_deg")
yaw_vr, yaw_type = GetVariableInfo(vars_, "yaw_angle_rad")
u_vr, u_type     = GetVariableInfo(vars_, "forward_speed")
cs_vr, cs_type   = GetVariableInfo(vars_, "current_speed")
cd_vr, cd_type   = GetVariableInfo(vars_, "current_dir_rad")

for nm, tp in [("rudder_angle_deg", rud_type), ("yaw_angle_rad", yaw_type), ("forward_speed", u_type),
               ("current_speed", cs_type), ("current_dir_rad", cd_type)]:
    if tp != CosimVariableType.REAL:
        raise RuntimeError(f"Expected REAL for {nm}, got {tp}")

def rudder_cmd_deg(t_s):
    # step sequence: 0 -> +10 -> -10 -> 0
    if t_s < 5:
        return 0.0
    elif t_s < 10:
        return 10.0
    elif t_s < 15:
        return -10.0
    else:
        return 0.0

def forward_speed(t_s):
    return 5.0  # constant 5 m/s

def yaw_angle(t_s):
    return 0.1 * np.sin(2*np.pi*0.05*t_s)  # small yaw oscillation [rad]

def current_speed(t_s):
    return 0.5  # m/s

def current_dir_rad(t_s):
    return float(np.deg2rad(30.0))  # 30 deg

# Initialize
manip.slave_real_values(idx, [rud_vr], [rudder_cmd_deg(0.0)])
manip.slave_real_values(idx, [yaw_vr], [yaw_angle(0.0)])
manip.slave_real_values(idx, [u_vr],   [forward_speed(0.0)])
manip.slave_real_values(idx, [cs_vr],  [current_speed(0.0)])
manip.slave_real_values(idx, [cd_vr],  [current_dir_rad(0.0)])

t = 0
while t < stopTime:
    ts = t / 1e9
    manip.slave_real_values(idx, [rud_vr], [rudder_cmd_deg(ts)])
    manip.slave_real_values(idx, [yaw_vr], [yaw_angle(ts)])
    manip.slave_real_values(idx, [u_vr],   [forward_speed(ts)])
    manip.slave_real_values(idx, [cs_vr],  [current_speed(ts)])
    manip.slave_real_values(idx, [cd_vr],  [current_dir_rad(ts)])
    execution.step()
    t += stepSize

plt.figure()
plot_real_ts(observer, idx, v_vr, "rudder_force_v")
plt.xlabel("Time [s]")
plt.ylabel("Sway force [N]")
plt.grid(True)
plt.legend()

plt.figure()
plot_real_ts(observer, idx, r_vr, "rudder_force_r")
plt.xlabel("Time [s]")
plt.ylabel("Yaw torque [Nm]")
plt.grid(True)
plt.legend()

plt.show()
