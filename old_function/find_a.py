import numpy as np

rpm_data   = np.array([ 66, 100, 200, 300, 400, 450, 500, 600, 660], dtype=float)
v_data     = np.array([0.18,0.407,1.533,3.168,5.14,6.2, 6.72,6.72,6.72], dtype=float)

# 1) Remove the saturated plateau (optional but recommended)
mask = v_data < 6.72  # keep strictly below plateau
rpm_lin = rpm_data[mask]
v_lin   = v_data[mask]

# # 2) Build speed->rpm feedforward via interpolation.
# # np.interp expects x to be increasing. Ensure v_lin is sorted (it is).
# def rpm_ff(v_des):
#     v_des = float(v_des)
#     # clamp to measured range to avoid extrapolation surprises
#     v_des = np.clip(v_des, v_lin.min(), v_lin.max())
#     return float(np.interp(v_des, v_lin, rpm_lin))

# # Example
# for v in [0.5, 1.0, 2.0, 4.0, 6.0]:
#     print(v, rpm_ff(v))
    
def a_local(v_query):
    # find which segment v_query falls into and return that segment slope
    vq = float(np.clip(v_query, v_lin.min(), v_lin.max()))
    idx = np.searchsorted(v_lin, vq) - 1
    idx = int(np.clip(idx, 0, len(v_lin)-2))
    dv = v_lin[idx+1] - v_lin[idx]
    dr = rpm_lin[idx+1] - rpm_lin[idx]
    return float(dr / dv)

for v in [0.5, 1.0, 2.0, 4.0, 6.0]:
    print(v, a_local(v), "rpm per (m/s)")

