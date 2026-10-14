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
from orchestrator.cosim_instance import GetVariableInfo, GetVariableIndex


def plot_real_ts(observer, slave_index, vr, label, sample_count=200000):
    t_ns, _, y = observer.time_series_real_samples(slave_index, value_reference=vr, sample_count=sample_count, from_step=0)
    t_s = np.array(t_ns) / 1e9
    plt.plot(t_s, y, label=label)

stepSize = int(1e9)     # 1.0 s
stopTime = int(100e9)   # 100 s (gives time to "move")

execution = CosimExecution.from_step_size(step_size=stepSize)
observer = CosimObserver.create_time_series()
execution.add_observer(observer=observer)

manip = CosimManipulator.create_override()
execution.add_manipulator(manipulator=manip)

fmu_path = str(ROOT / "FMUs" / "complex_ship" / "DopplerRADAR.fmu")
slave = CosimLocalSlave(fmu_path=fmu_path, instance_name="RADAR")
idx = execution.add_local_slave(local_slave=slave)
vars_ = execution.slave_variables(slave_index=idx)


# Use try/except so script runs even if some params differ.
try:
    execution.real_initial_value(idx, GetVariableIndex(vars_, "max_target_ship_count"), 3)
    execution.real_initial_value(idx, GetVariableIndex(vars_, "range_sigma"), 10.0)
    execution.real_initial_value(idx, GetVariableIndex(vars_, "range_rate_sigma"), 0.2)
    execution.real_initial_value(idx, GetVariableIndex(vars_, "bearing_sigma_deg"), 1.0)
    execution.real_initial_value(idx, GetVariableIndex(vars_, "max_range"), 1000.0)
except Exception:
    pass

# Outputs
measurement_valid_vr, measurement_valid_type    = GetVariableInfo(vars_, "measurement_valid")

detect_tar_1_vr, detect_tar_1_type              = GetVariableInfo(vars_, "detect_tar_1")
tar_1_range_vr, tar_1_range_type                = GetVariableInfo(vars_, "tar_1_range")
tar_1_range_rate_vr, tar_1_range_rate_type      = GetVariableInfo(vars_, "tar_1_range_rate")
tar_1_bearing_vr, tar_1_bearing_type            = GetVariableInfo(vars_, "tar_1_bearing")
observer.start_time_series(idx, value_reference=detect_tar_1_vr, variable_type=detect_tar_1_type)
observer.start_time_series(idx, value_reference=tar_1_range_vr, variable_type=tar_1_range_type)
observer.start_time_series(idx, value_reference=tar_1_range_rate_vr, variable_type=tar_1_range_rate_type)
observer.start_time_series(idx, value_reference=tar_1_bearing_vr, variable_type=tar_1_bearing_type)

detect_tar_2_vr, detect_tar_2_type          = GetVariableInfo(vars_, "detect_tar_2")
tar_2_range_vr, tar_2_range_type            = GetVariableInfo(vars_, "tar_2_range")
tar_2_range_rate_vr, tar_2_range_rate_type  = GetVariableInfo(vars_, "tar_2_range_rate")
tar_2_bearing_vr, tar_2_bearing_type        = GetVariableInfo(vars_, "tar_2_bearing")
observer.start_time_series(idx, value_reference=detect_tar_2_vr, variable_type=detect_tar_2_type)
observer.start_time_series(idx, value_reference=tar_2_range_vr, variable_type=tar_2_range_type)
observer.start_time_series(idx, value_reference=tar_2_range_rate_vr, variable_type=tar_2_range_rate_type)
observer.start_time_series(idx, value_reference=tar_2_bearing_vr, variable_type=tar_2_bearing_type)

detect_tar_3_vr, detect_tar_3_type          = GetVariableInfo(vars_, "detect_tar_3")
tar_3_range_vr, tar_3_range_type            = GetVariableInfo(vars_, "tar_3_range")
tar_3_range_rate_vr, tar_3_range_rate_type  = GetVariableInfo(vars_, "tar_3_range_rate")
tar_3_bearing_vr, tar_3_bearing_type        = GetVariableInfo(vars_, "tar_3_bearing")
observer.start_time_series(idx, value_reference=detect_tar_3_vr, variable_type=detect_tar_3_type)
observer.start_time_series(idx, value_reference=tar_3_range_vr, variable_type=tar_3_range_type)
observer.start_time_series(idx, value_reference=tar_3_range_rate_vr, variable_type=tar_3_range_rate_type)
observer.start_time_series(idx, value_reference=tar_3_bearing_vr, variable_type=tar_3_bearing_type)

last_observer = CosimObserver.create_last_value()
execution.add_observer(observer=last_observer)

# Inputs
radar_degradation_vr, _         = GetVariableInfo(vars_, "radar_degradation")

own_north_vr, _                 = GetVariableInfo(vars_, "own_north")
own_east_vr, _                  = GetVariableInfo(vars_, "own_east")
own_yaw_angle_vr, _             = GetVariableInfo(vars_, "own_yaw_angle")
own_measured_speed_vr, _        = GetVariableInfo(vars_, "own_measured_speed")

tar_1_north_vr, _               = GetVariableInfo(vars_, "tar_1_north")
tar_1_east_vr, _                = GetVariableInfo(vars_, "tar_1_east")
tar_1_yaw_angle_vr, _           = GetVariableInfo(vars_, "tar_1_yaw_angle")
tar_1_measured_speed_vr, _      = GetVariableInfo(vars_, "tar_1_measured_speed")

tar_2_north_vr, _               = GetVariableInfo(vars_, "tar_2_north")
tar_2_east_vr, _                = GetVariableInfo(vars_, "tar_2_east")
tar_2_yaw_angle_vr, _           = GetVariableInfo(vars_, "tar_2_yaw_angle")
tar_2_measured_speed_vr, _      = GetVariableInfo(vars_, "tar_2_measured_speed")

tar_3_north_vr, _               = GetVariableInfo(vars_, "tar_3_north")
tar_3_east_vr, _                = GetVariableInfo(vars_, "tar_3_east")
tar_3_yaw_angle_vr, _           = GetVariableInfo(vars_, "tar_3_yaw_angle")
tar_3_measured_speed_vr, _      = GetVariableInfo(vars_, "tar_3_measured_speed")

radar_input_vrs = [
    # measurement_valid_vr
    # radar_degradation_vr,

    own_north_vr,
    own_east_vr,
    own_yaw_angle_vr,
    own_measured_speed_vr,

    tar_1_north_vr,
    tar_1_east_vr,
    tar_1_yaw_angle_vr,
    tar_1_measured_speed_vr,

    tar_2_north_vr,
    tar_2_east_vr,
    tar_2_yaw_angle_vr,
    tar_2_measured_speed_vr,

    tar_3_north_vr,
    tar_3_east_vr,
    tar_3_yaw_angle_vr,
    tar_3_measured_speed_vr,
]

# ================================================================
# TEST SCENARIO
# ================================================================
# Own ship
o_n_0   = 0.0
o_e_0   = 0.0
o_vn    = 5.0
o_ve    = 0.0
o_psi_0 = np.arctan2(o_ve, o_vn)   # 0 rad = North

# Target 1:
# Head-on / opposing course.
#
# Small 25 m lateral offset prevents exact R = 0 at CPA.
t1_n_0   = 800.0
t1_e_0   = 25.0
t1_vn    = -5.0
t1_ve    = 0.0
t1_psi_0 = np.arctan2(t1_ve, t1_vn)

# Target 2:
# Initially west of own ship and keeps moving west.
# Therefore range monotonically increases.
t2_n_0   = 0.0
t2_e_0   = 1200.0
t2_vn    = 0.0
t2_ve    = -5.0
t2_psi_0 = np.arctan2(t2_ve, t2_vn)

# Target 3:
# Starts southwest / aft-port of own ship.
# Travels diagonally NE faster than own ship.
t3_n_0   = -500.0
t3_e_0   = -400.0
t3_vn    = 7.0
t3_ve    = 4.0
t3_psi_0 = np.arctan2(t3_ve, t3_vn)

def constant_velocity_ship(t_s, n0, e0, vn, ve, psi0):
    """
    Constant-velocity ship trajectory.
    NED horizontal convention:
        psi = 0       -> North
        psi = +pi/2   -> East
        psi = -pi/2   -> West
    """
    north = n0 + vn * t_s
    east  = e0 + ve * t_s

    # No turning in this test
    yaw_angle = psi0

    speed = np.hypot(vn, ve)

    return (
        float(north),
        float(east),
        float(yaw_angle),
        float(speed),
        float(vn),
        float(ve),
    )

def own_ship_states(t_s):
    return constant_velocity_ship(
        t_s,
        o_n_0,
        o_e_0,
        o_vn,
        o_ve,
        o_psi_0,
    )

def target_ship_1_states(t_s):
    return constant_velocity_ship(
        t_s,
        t1_n_0,
        t1_e_0,
        t1_vn,
        t1_ve,
        t1_psi_0,
    )

def target_ship_2_states(t_s):
    return constant_velocity_ship(
        t_s,
        t2_n_0,
        t2_e_0,
        t2_vn,
        t2_ve,
        t2_psi_0,
    )

def target_ship_3_states(t_s):
    return constant_velocity_ship(
        t_s,
        t3_n_0,
        t3_e_0,
        t3_vn,
        t3_ve,
        t3_psi_0,
    )
    
def wrap_to_pi(angle):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi

def true_radar_measurement(own_state, target_state):
    """
    Analytical ground-truth radar measurement.

    Returns
    -------
    range_true      : m
    range_rate_true : m/s
                      negative = closing
                      positive = opening
    bearing_true    : rad, relative to own-ship heading
    """
    o_n, o_e, o_psi, _, o_vn, o_ve = own_state
    t_n, t_e, _,     _, t_vn, t_ve = target_state

    dn = t_n - o_n
    de = t_e - o_e

    dvn = t_vn - o_vn
    dve = t_ve - o_ve

    r = np.hypot(dn, de)

    if r < 1e-9:
        range_rate = np.nan
        bearing = np.nan
    else:
        range_rate = (dn*dvn + de*dve) / r

        los_angle = np.arctan2(de, dn)

        bearing = wrap_to_pi(
            los_angle - o_psi
        )
    return r, range_rate, bearing

def wrap_to_pi(angle):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi

def true_radar_measurement(own_state, target_state):
    """
    Analytical ground-truth radar measurement.

    Returns
    -------
    range_true      : m
    range_rate_true : m/s
                      negative = closing
                      positive = opening
    bearing_true    : rad, relative to own-ship heading
    """

    o_n, o_e, o_psi, _, o_vn, o_ve = own_state
    t_n, t_e, _,     _, t_vn, t_ve = target_state

    dn = t_n - o_n
    de = t_e - o_e

    dvn = t_vn - o_vn
    dve = t_ve - o_ve

    r = np.hypot(dn, de)

    if r < 1e-9:
        range_rate = np.nan
        bearing = np.nan
    else:
        range_rate = (dn*dvn + de*dve) / r

        los_angle = np.arctan2(de, dn)

        bearing = wrap_to_pi(
            los_angle - o_psi
        )

    return r, range_rate, bearing

def read_detection(observer, slave_idx, vr, variable_type):
    if variable_type == CosimVariableType.BOOLEAN:
        return bool(
            observer.last_boolean_values(
                slave_index=slave_idx,
                variable_references=[vr]
            )[0]
        )

    elif variable_type == CosimVariableType.INTEGER:
        return bool(
            observer.last_integer_values(
                slave_index=slave_idx,
                variable_references=[vr]
            )[0]
        )

    elif variable_type == CosimVariableType.REAL:
        return bool(
            observer.last_real_values(
                slave_index=slave_idx,
                variable_references=[vr]
            )[0]
        )

    else:
        raise ValueError(
            f"Unsupported detection variable type: {variable_type}"
        )

def set_radar_inputs(t_s, radar_degradation=False):
    own = own_ship_states(t_s)
    t1  = target_ship_1_states(t_s)
    t2  = target_ship_2_states(t_s)
    t3  = target_ship_3_states(t_s)

    own_n, own_e, own_psi, own_speed, _, _ = own
    t1_n,  t1_e,  t1_psi,  t1_speed,  _, _ = t1
    t2_n,  t2_e,  t2_psi,  t2_speed,  _, _ = t2
    t3_n,  t3_e,  t3_psi,  t3_speed,  _, _ = t3

    values = [
        # 0.0,            # radar_degradation

        own_n,
        own_e,
        own_psi,
        own_speed,

        t1_n,
        t1_e,
        t1_psi,
        t1_speed,

        t2_n,
        t2_e,
        t2_psi,
        t2_speed,

        t3_n,
        t3_e,
        t3_psi,
        t3_speed,
    ]
    
    success = manip.slave_real_values(
        slave_index=idx,
        variable_references=radar_input_vrs,
        values=values,
    )
    
    manip.slave_boolean_values(
        slave_index=idx,
        variable_references=[radar_degradation_vr],
        values=[radar_degradation],
    )

    if not success:
        raise RuntimeError(
            f"Failed setting radar inputs at t={t_s:.2f}"
        )

    return own, t1, t2, t3

# ================================================================
# SIMULATION
# ================================================================
dt = stepSize / 1e9
n_steps = int(stopTime // stepSize)

# Measurement logs
time_log = []

det1_log = []
det2_log = []
det3_log = []

range_meas = [[], [], []]
rr_meas    = [[], [], []]
bearing_meas = [[], [], []]

range_true = [[], [], []]
rr_true    = [[], [], []]
bearing_true = [[], [], []]

radar_real_output_vrs = [
    tar_1_range_vr,
    tar_1_range_rate_vr,
    tar_1_bearing_vr,

    tar_2_range_vr,
    tar_2_range_rate_vr,
    tar_2_bearing_vr,

    tar_3_range_vr,
    tar_3_range_rate_vr,
    tar_3_bearing_vr,
]

for k in range(n_steps):
    t_s = k * dt

    # ------------------------------------------------------------
    # Drive radar FMU inputs
    # ------------------------------------------------------------
    own, t1, t2, t3 = set_radar_inputs(t_s)

    # ------------------------------------------------------------
    # Analytical truth
    # ------------------------------------------------------------
    truths = [
        true_radar_measurement(own, t1),
        true_radar_measurement(own, t2),
        true_radar_measurement(own, t3),
    ]

    # Check if it still a sensor placement at first
    measurement_valid   = read_detection(last_observer,
                                         idx,
                                         measurement_valid_vr,
                                         measurement_valid_type)
    
    # ------------------------------------------------------------
    # Advance FMU
    # ------------------------------------------------------------
    success = execution.step()
    if not success:
        raise RuntimeError(
            f"Co-simulation step failed at t={t_s:.2f} s"
        )
    
    if not measurement_valid:
        continue

    # ------------------------------------------------------------
    # Read radar measurements
    # ------------------------------------------------------------
    y = last_observer.last_real_values(
        slave_index=idx,
        variable_references=radar_real_output_vrs,
    )
    
    # y =
    # [
    #    t1 range,
    #    t1 range rate,
    #    t1 bearing,
    #    t2 range,
    #    ...
    # ]
    
    detections = [
        read_detection(
            last_observer,
            idx,
            detect_tar_1_vr,
            detect_tar_1_type,
        ),
        read_detection(
            last_observer,
            idx,
            detect_tar_2_vr,
            detect_tar_2_type,
        ),
        read_detection(
            last_observer,
            idx,
            detect_tar_3_vr,
            detect_tar_3_type,
        ),
    ]

    # ------------------------------------------------------------
    # Log
    # ------------------------------------------------------------
    time_log.append(t_s)

    det1_log.append(detections[0])
    det2_log.append(detections[1])
    det3_log.append(detections[2])
    
    for i in range(3):
        j = 3 * i

        if detections[i] is True:
            range_meas[i].append(y[j])
            rr_meas[i].append(y[j + 1])
            bearing_meas[i].append(y[j + 2])
        else:
            range_meas[i].append(np.nan)
            rr_meas[i].append(np.nan)
            bearing_meas[i].append(np.nan)

        range_true[i].append(truths[i][0])
        rr_true[i].append(truths[i][1])
        bearing_true[i].append(truths[i][2])

print("Simulation complete.")

# PLOT
t_plot = np.linspace(0.0, 100.0, 501)
own_xy = np.array([
    own_ship_states(t)[:2]
    for t in t_plot
])
t1_xy = np.array([
    target_ship_1_states(t)[:2]
    for t in t_plot
])
t2_xy = np.array([
    target_ship_2_states(t)[:2]
    for t in t_plot
])
t3_xy = np.array([
    target_ship_3_states(t)[:2]
    for t in t_plot
])


plt.figure(figsize=(8, 8))
plt.plot(
    own_xy[:, 1],
    own_xy[:, 0],
    label="Own ship",
)
plt.plot(
    t1_xy[:, 1],
    t1_xy[:, 0],
    label="Target 1: head-on",
)
plt.plot(
    t2_xy[:, 1],
    t2_xy[:, 0],
    label="Target 2: receding",
)
plt.plot(
    t3_xy[:, 1],
    t3_xy[:, 0],
    label="Target 3: aft diagonal",
)
plt.scatter(
    [
        own_xy[0,1],
        t1_xy[0,1],
        t2_xy[0,1],
        t3_xy[0,1],
    ],
    [
        own_xy[0,0],
        t1_xy[0,0],
        t2_xy[0,0],
        t3_xy[0,0],
    ],
    marker="o",
)

plt.xlabel("East [m]")
plt.ylabel("North [m]")
plt.title("Radar Test Scenario")
plt.axis("equal")
plt.grid()
plt.legend()

# ================================================================
# Plot Measurement
# ================================================================

# # Measurement logs
# time_log = []

# det1_log = []
# det2_log = []
# det3_log = []

# range_meas = [[], [], []]
# rr_meas    = [[], [], []]
# bearing_meas = [[], [], []]

# range_true = [[], [], []]
# rr_true    = [[], [], []]
# bearing_true = [[], [], []]

label       =["TS1", "TS2", "TS3"]
label_truth =["TS1_truth", "TS2_truth", "TS3_truth"]

# Actual measurement
plt.figure(figsize=(8, 8))
all_detection = [det1_log, det2_log, det3_log]
for i, detection in enumerate(all_detection):
    plt.plot(time_log, detection, label=label[i])
    plt.xlabel("Time [s]")
    plt.ylabel("True/False")
    plt.yticks([0, 1], ["False", "True"])
    plt.ylim(-0.1, 1.1)
    plt.grid()
    plt.legend()

plt.figure(figsize=(8, 8))
for i, (range_mea, range_tru) in enumerate(zip(range_meas, range_true)):
    plt.plot(time_log, range_mea, label=label[i])
    plt.plot(time_log, range_tru, label=label_truth[i])
    plt.xlabel("Time [s]")
    plt.ylabel("Range [m]")
    plt.grid()
    plt.legend()

plt.figure(figsize=(8, 8))
for i, (rr_mea, rr_tru) in enumerate(zip(rr_meas, rr_true)):
    plt.plot(time_log, rr_mea, label=label[i])
    plt.plot(time_log, rr_tru, label=label_truth[i])
    plt.xlabel("Time [s]")
    plt.ylabel("Range rate [m/s]")
    plt.grid()
    plt.legend()
    
plt.figure(figsize=(8, 8))
for i, (bearing_mea, bearing_tru) in enumerate(zip(bearing_meas, bearing_true)):
    plt.plot(time_log, bearing_mea, label=label[i])
    plt.plot(time_log, bearing_tru, label=label_truth[i])
    plt.xlabel("Time [s]")
    plt.ylabel("Bearing [deg]")
    plt.grid()
    plt.legend()

plt.show()