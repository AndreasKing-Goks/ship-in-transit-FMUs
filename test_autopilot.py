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
import numpy as np

from libcosimpy.CosimExecution import CosimExecution
from libcosimpy.CosimSlave import CosimLocalSlave
from libcosimpy.CosimManipulator import CosimManipulator
from libcosimpy.CosimObserver import CosimObserver
from libcosimpy.CosimEnums import CosimVariableType
from utils import GetVariableInfo, GetVariableIndex


def plot_real_ts(observer, slave_index, vr, label, sample_count=200000):
    t_ns, _, y = observer.time_series_real_samples(slave_index, value_reference=vr, sample_count=sample_count, from_step=0)
    t_s = np.array(t_ns) / 1e9
    plt.plot(t_s, y, label=label)

stepSize = int(1e7)     # 0.01 s
stopTime = int(100e9)    # 40 s (gives time to "move")

execution = CosimExecution.from_step_size(step_size=stepSize)
observer = CosimObserver.create_time_series()
execution.add_observer(observer=observer)

manip = CosimManipulator.create_override()
execution.add_manipulator(manipulator=manip)

fmu_path = str(ROOT / "FMUs" / "Autopilot.fmu")
slave = CosimLocalSlave(fmu_path=fmu_path, instance_name="AUTOPILOT")
idx = execution.add_local_slave(local_slave=slave)
vars_ = execution.slave_variables(slave_index=idx)

# Optional: set autopilot parameters if present (adapt to your FMU names)
# Use try/except so script runs even if some params differ.
try:
    execution.real_initial_value(idx, GetVariableIndex(vars_, "kp"), 10.5)
    execution.real_initial_value(idx, GetVariableIndex(vars_, "ki"), 0.0)
    execution.real_initial_value(idx, GetVariableIndex(vars_, "kd"), 0.5)
    execution.real_initial_value(idx, GetVariableIndex(vars_, "max_rudder_rate_deg_per_sec"), 2.3)
    execution.real_initial_value(idx, GetVariableIndex(vars_, "max_rudder_angle_deg"), 35.0)
    execution.real_initial_value(idx, GetVariableIndex(vars_, "integrator_limit"), 4000.0)
except Exception:
    pass

# Outputs
yawref_vr, yawref_type = GetVariableInfo(vars_, "yaw_angle_ref_rad")
rud_vr, rud_type       = GetVariableInfo(vars_, "rudder_angle_deg")
ect_vr, ect_type       = GetVariableInfo(vars_, "e_ct")
observer.start_time_series(idx, value_reference=yawref_vr, variable_type=yawref_type)
observer.start_time_series(idx, value_reference=rud_vr, variable_type=rud_type)
observer.start_time_series(idx, value_reference=ect_vr, variable_type=ect_type)

# Inputs
north_vr, _ = GetVariableInfo(vars_, "north")
east_vr, _  = GetVariableInfo(vars_, "east")
yaw_vr, _   = GetVariableInfo(vars_, "yaw_angle_rad")

nwpn_vr, _  = GetVariableInfo(vars_, "next_wp_north")
nwpe_vr, _  = GetVariableInfo(vars_, "next_wp_east")
pwpn_vr, _  = GetVariableInfo(vars_, "prev_wp_north")
pwpe_vr, _  = GetVariableInfo(vars_, "prev_wp_east")

# Simple “fake ship motion”: move roughly along x=y line with a lateral offset to create cross-track error.
def ship_position(t_s):
    v = 5.0  # m/s
    north = v * t_s
    east  = v * t_s + 50.0 * np.sin(2*np.pi*0.02*t_s)  # weave +/-50m slowly
    return float(north), float(east)

def ship_yaw(t_s):
    # crude yaw estimate: align roughly with path (45deg) + small perturbation
    return float(np.deg2rad(45.0) + 0.05*np.sin(2*np.pi*0.03*t_s))

# Fixed waypoints (prev->next)
prev_wp = (0.0, 0.0)
next_wp = (2000.0, 2000.0)

# Init inputs
manip.slave_real_values(idx, [pwpn_vr], [prev_wp[0]])
manip.slave_real_values(idx, [pwpe_vr], [prev_wp[1]])
manip.slave_real_values(idx, [nwpn_vr], [next_wp[0]])
manip.slave_real_values(idx, [nwpe_vr], [next_wp[1]])

n0, e0 = ship_position(0.0)
manip.slave_real_values(idx, [north_vr], [n0])
manip.slave_real_values(idx, [east_vr], [e0])
manip.slave_real_values(idx, [yaw_vr], [ship_yaw(0.0)])

t = 0
while t < stopTime:
    ts = t / 1e9
    n, e = ship_position(ts)
    manip.slave_real_values(idx, [north_vr], [n])
    manip.slave_real_values(idx, [east_vr], [e])
    manip.slave_real_values(idx, [yaw_vr], [ship_yaw(ts)])
    execution.step()
    t += stepSize

plt.figure()
plot_real_ts(observer, idx, yawref_vr, "yaw_angle_ref_rad")
plt.xlabel("Time [s]")
plt.ylabel("Yaw ref [rad]")
plt.grid(True)
plt.legend()

plt.figure()
plot_real_ts(observer, idx, rud_vr, "rudder_angle_deg")
plt.xlabel("Time [s]")
plt.ylabel("Rudder angle [deg]")
plt.grid(True)
plt.legend()

plt.figure()
plot_real_ts(observer, idx, ect_vr, "e_ct")
plt.xlabel("Time [s]")
plt.ylabel("Cross-track error [m]")
plt.grid(True)
plt.legend()

plt.show()