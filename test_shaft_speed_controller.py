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


# -----------------------
# Sim settings (ns)
# -----------------------
stepSize = int(1e7)     # 0.01 s
stopTime = int(20e9)    # 20 s

execution = CosimExecution.from_step_size(step_size=stepSize)

observer = CosimObserver.create_time_series()
execution.add_observer(observer=observer)

manip = CosimManipulator.create_override()
execution.add_manipulator(manipulator=manip)

# -----------------------
# Add FMU
# -----------------------
fmu_path = str(ROOT / "FMUs" / "ShaftSpeedController.fmu")
slave = CosimLocalSlave(fmu_path=fmu_path, instance_name="SHAFT_SPEED_CONTROLLER")
idx = execution.add_local_slave(local_slave=slave)
vars_ = execution.slave_variables(slave_index=idx)

# -----------------------
# (Optional) Set gains if exposed as params
# -----------------------
# If your controller FMU registers kp/ki as parameters, you can set them here.
# Comment these out if your FMU doesn't have them.
try:
    kp_vr = GetVariableIndex(vars_, "kp")
    ki_vr = GetVariableIndex(vars_, "ki")
    execution.real_initial_value(idx, kp_vr, 2.5)
    execution.real_initial_value(idx, ki_vr, 0.025)
except Exception:
    pass

# -----------------------
# Outputs to observe
# -----------------------
cmd_vr, cmd_type = GetVariableInfo(vars_, "shaft_speed_cmd_rpm")
observer.start_time_series(idx, value_reference=cmd_vr, variable_type=cmd_type)

# -----------------------
# Input VRs
# -----------------------
des_vr, des_type = GetVariableInfo(vars_, "desired_ship_speed")
mea_vr, mea_type = GetVariableInfo(vars_, "measured_ship_speed")

if des_type != CosimVariableType.REAL or mea_type != CosimVariableType.REAL:
    raise RuntimeError(f"Expected REAL inputs. Got desired={des_type}, measured={mea_type}")

# -----------------------
# Helpers
# -----------------------
def desired_ship_speed(t_s: float) -> float:
    if t_s < 5.0:
        return 3.0
    elif t_s < 10.0:
        return 5.0
    elif t_s < 15.0:
        return 7.0
    else:
        return 4.0

def measured_ship_speed(t_s: float) -> float:
    # ramp + small wobble (always positive)
    base = min(8.0, 0.6 * t_s)             # 0.6 m/s^2 ramp until 8 m/s
    wobble = 0.1 * np.sin(2*np.pi*0.2*t_s)
    return float(max(0.0, base + wobble))

# Initialize inputs before first step
manip.slave_real_values(idx, [des_vr], [desired_ship_speed(0.0)])
manip.slave_real_values(idx, [mea_vr], [measured_ship_speed(0.0)])

# -----------------------
# Run
# -----------------------
t = 0
while t < stopTime:
    ts = t / 1e9
    manip.slave_real_values(idx, [des_vr], [desired_ship_speed(ts)])
    manip.slave_real_values(idx, [mea_vr], [measured_ship_speed(ts)])
    execution.step()
    t += stepSize

# -----------------------
# Plot
# -----------------------
plt.figure()
plot_real_ts(observer, idx, cmd_vr, "shaft_speed_cmd_rpm")
plt.xlabel("Time [s]")
plt.ylabel("Shaft speed command [RPM]")
plt.grid(True)
plt.legend()
plt.show()
