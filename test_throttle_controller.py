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
from utils import *  # assumes GetVariableIndex / GetVariableInfo exist


# ============================================================
# Throttle Controller Standalone Test
#   - drives desired_shaft_speed_rpm with steps
#   - feeds measured_shaft_speed_rpm as a synthetic signal
#   - observes throttle_cmd
# ============================================================

# Use integer nanoseconds everywhere
stepSize = int(1e7)      # 0.01 s
stopTime = int(20e9)     # 20 s

execution = CosimExecution.from_step_size(step_size=stepSize)

# Observer
observe_time_series = CosimObserver.create_time_series()
execution.add_observer(observer=observe_time_series)

# Manipulator
manipulator = CosimManipulator.create_override()
execution.add_manipulator(manipulator=manipulator)

# Add ThrottleController FMU
throttle_fmu_path = str(ROOT / "FMUs" / "ThrottleController.fmu")
throttle_slave = CosimLocalSlave(fmu_path=throttle_fmu_path, instance_name="THROTTLE_CONTROLLER")
throttle_index = execution.add_local_slave(local_slave=throttle_slave)
throttle_vars = execution.slave_variables(slave_index=throttle_index)

# ------------------------------------------------------------
# Set controller gains (edit as needed)
# ------------------------------------------------------------
kp_val = 0.05
ki_val = 0.0001

kp_vr = GetVariableIndex(throttle_vars, "kp")
ki_vr = GetVariableIndex(throttle_vars, "ki")
execution.real_initial_value(throttle_index, kp_vr, float(kp_val))
execution.real_initial_value(throttle_index, ki_vr, float(ki_val))

# ------------------------------------------------------------
# Register observer for throttle_cmd
# ------------------------------------------------------------
throttle_cmd_vr, throttle_cmd_type = GetVariableInfo(throttle_vars, "throttle_cmd")
observe_time_series.start_time_series(
    throttle_index, value_reference=throttle_cmd_vr, variable_type=throttle_cmd_type
)

# ------------------------------------------------------------
# Input VRs (desired / measured)
# ------------------------------------------------------------
desired_vr, desired_type = GetVariableInfo(throttle_vars, "desired_shaft_speed_rpm")
measured_vr, measured_type = GetVariableInfo(throttle_vars, "measured_shaft_speed_rpm")

if desired_type != CosimVariableType.REAL or measured_type != CosimVariableType.REAL:
    raise RuntimeError(f"Expected REAL inputs. Got desired={desired_type}, measured={measured_type}")

# ------------------------------------------------------------
# Helper: generate synthetic measured shaft speed signal
# ------------------------------------------------------------
def measured_signal(t_s: float) -> float:
    """
    Simple, stable test signal:
      - starts at 0
      - ramps up, then oscillates slightly
    """
    base = min(600.0, 30.0 * t_s)            # ramp 30 rpm/s up to 600 rpm
    wobble = 10.0 * np.sin(2.0 * np.pi * 0.2 * t_s)  # small 0.2 Hz wobble
    return float(base + wobble)

def desired_signal(t_s: float) -> float:
    """
    Step schedule for desired shaft speed.
    """
    if t_s < 5.0:
        return 200.0
    elif t_s < 10.0:
        return 350.0
    elif t_s < 15.0:
        return 550.0
    else:
        return 300.0

# ------------------------------------------------------------
# Initialize inputs before first step
# ------------------------------------------------------------
manipulator.slave_real_values(throttle_index, [desired_vr], [desired_signal(0.0)])
manipulator.slave_real_values(throttle_index, [measured_vr], [measured_signal(0.0)])

# Run
time_ns = 0
while time_ns < stopTime:
    t_s = time_ns / 1e9

    desired = desired_signal(t_s)
    measured = measured_signal(t_s)

    # Set inputs
    manipulator.slave_real_values(throttle_index, [desired_vr], [desired])
    manipulator.slave_real_values(throttle_index, [measured_vr], [measured])

    # Step
    execution.step()
    time_ns += stepSize

# ------------------------------------------------------------
# Fetch results and plot
# ------------------------------------------------------------
t_ns, _, throttle_samples = observe_time_series.time_series_real_samples(
    throttle_index, value_reference=throttle_cmd_vr, sample_count=200000, from_step=0
)

t_s = [x / 1e9 for x in t_ns]

# Also plot desired/measured signals (recomputed for plotting)
desired_plot = [desired_signal(ts) for ts in t_s]
measured_plot = [measured_signal(ts) for ts in t_s]

plt.figure()
plt.plot(t_s, desired_plot, label="desired_shaft_speed_rpm")
plt.plot(t_s, measured_plot, label="measured_shaft_speed_rpm")
plt.xlabel("Time [s]")
plt.ylabel("Shaft Speed [RPM]")
plt.grid(True)
plt.legend()

plt.figure()
plt.plot(t_s, throttle_samples, label="throttle_cmd")
plt.xlabel("Time [s]")
plt.ylabel("Throttle [-]")
plt.grid(True)
plt.legend()

plt.show()