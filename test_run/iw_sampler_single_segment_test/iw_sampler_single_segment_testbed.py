import numpy as np
import matplotlib.pyplot as plt
import math

# Base
base_north              = np.float64(0.0)
base_east               = np.float64(0.0)

end_north               = np.float64(100.0)
end_east                = np.float64(100.0)

segment_north           = [base_north, end_north]
segment_east            = [base_east , end_east ]

d_segment_north         = end_north - base_north
d_segment_east          = end_east - base_east

segment_length          = np.hypot(d_segment_north, d_segment_east)

beta                    = np.atan2(d_segment_east, d_segment_north)

######################## TEST_FUNCTION ########################

# Zeroeth IWP
def _get_iw_sampler_component(n_iw, e_iw):
    # Base to IW component lengths
    d_base_to_iw_north = n_iw - base_north
    d_base_to_iw_east  = e_iw  - base_east
    
    # Base to IW length
    base_iw_length     = np.hypot(d_base_to_iw_north, d_base_to_iw_east)
    
    # Angle from east axis to the IW line
    beta_iw            = np.atan2(d_base_to_iw_east, d_base_to_iw_north)
    
    # Angle from the route segment to the IW line
    psi                = beta_iw - beta
    
    # Traversed Segment Length
    traversed_segment_length    = base_iw_length * np.cos(psi)
    segment_arm_length          = base_iw_length * np.sin(psi)
    untraversed_segment_length  = segment_length - traversed_segment_length
    
    # IW to route projection coordinate
    # print("base_north: ", base_north)
    # print("base_east: ", base_east)
    # print("base_iw_length: ", base_iw_length)
    # print("traversed_segment_length: ", traversed_segment_length)
    # print("psi: ", np.rad2deg(psi))
    # print("d_tsl_n: ", traversed_segment_length*np.cos(psi))
    # print("d_tsl_e: ", traversed_segment_length*np.sin(psi))
    p_north                     = base_north + traversed_segment_length * np.cos(beta)
    p_east                      = base_east  + traversed_segment_length * np.sin(beta)
    
    return (traversed_segment_length,
            untraversed_segment_length,
            segment_arm_length,
            p_north,
            p_east)
    
def _scope_next_iw(scope_angle_deg, scope_length, prev_wp_north, prev_wp_east):
    # Convert scope angle deg to rad
    psi         = np.deg2rad(scope_angle_deg)
    
    # Get angle form east axist to the IW line
    beta_iw     = beta + psi
    
    # Get the d_north and d_east
    d_north     = scope_length * np.cos(beta_iw)
    d_east      = scope_length * np.sin(beta_iw)
    
    # Get the next waypoint
    next_wp_north = prev_wp_north + d_north
    next_wp_east  = prev_wp_east  + d_east
    
    return (next_wp_north,
            next_wp_east)
    
######################## TEST_FUNCTION ########################
n_ship                  = np.float64(10.0)
e_ship                  = np.float64(20.0)

## Container for plottings
# Intermediate Waypoints
n_iw_list = []
e_iw_list = []

# Ship trajectory
traj_n      = [base_north]
traj_e      = [base_east]

# IW Projection
p_n_list = []
p_e_list = []

# IW sampler components
traversed_segment_length_list   = []
untraversed_segment_length_list = []
segment_arm_length_list         = []

# Commands
scope_angles_deg = [30, -30, -30, -15, 0]
scope_length    = 25

# FIRST TRIGGER
# Set the current ship position as IW0
n_iw_0                  = n_ship
e_iw_0                  = e_ship

# Get the IW sampler component
(traversed_segment_length_0, 
untraversed_segment_length_0,
segment_arm_length_0, 
p_n_0, p_e_0) = _get_iw_sampler_component(n_iw_0, e_iw_0)

# Store
traversed_segment_length_list.append(traversed_segment_length_0)
untraversed_segment_length_list.append(untraversed_segment_length_0)
segment_arm_length_list.append(segment_arm_length_0)
p_n_list.append(p_n_0)
p_e_list.append(p_e_0)
n_iw_list.append(n_iw_0)
e_iw_list.append(e_iw_0)
traj_n.append(n_iw_0)
traj_e.append(e_iw_0)

# Loop
for i, scope_angle_deg in enumerate(scope_angles_deg):
    # Get the next waypoint
    n_iw, e_iw = _scope_next_iw(scope_angle_deg, scope_length, n_iw_list[i], e_iw_list[i])
    
    # Get the IW sampler component
    (traversed_segment_length, 
    untraversed_segment_length,
    segment_arm_length, 
    p_n, p_e) = _get_iw_sampler_component(n_iw, e_iw)
    
    # Store
    traversed_segment_length_list.append(traversed_segment_length)
    untraversed_segment_length_list.append(untraversed_segment_length)
    segment_arm_length_list.append(segment_arm_length)
    p_n_list.append(p_n)
    p_e_list.append(p_e)
    n_iw_list.append(n_iw)
    e_iw_list.append(e_iw)
    traj_n.append(n_iw)
    traj_e.append(e_iw)
    
    if untraversed_segment_length < scope_length:
        break
    
# Add endpoint
traj_n.append(end_north)
traj_e.append(end_east)

# Check
length_list = []
for i in range(len(e_iw_list)):
    if i >= (len(e_iw_list)-1):
        break
    length = np.hypot((e_iw_list[i+1]-e_iw_list[i]), (n_iw_list[i+1]-n_iw_list[i]))
    length_list.append(length)
    
print(length_list)

######################## PLOT ########################
# Figure
plt.figure()

# Segment
plt.plot(segment_east, segment_north)
plt.scatter(segment_east, segment_north)

# IW
for i in range(len(e_iw_list)):
    plt.scatter(e_iw_list[i], n_iw_list[i])
    plt.scatter(p_e_list[i], p_n_list[i])
    plt.plot([p_e_list[i], e_iw_list[i]], [p_n_list[i], n_iw_list[i]])
    
# Trajectories
plt.plot(traj_e, traj_n)

plt.grid()
plt.title("IW Sampler")
plt.gca().set_aspect('equal', adjustable='box')
# plt.xlim([-5, 105])
# plt.ylim([-5, 105])

plt.show()
