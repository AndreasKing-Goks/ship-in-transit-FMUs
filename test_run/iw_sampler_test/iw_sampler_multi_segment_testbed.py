import numpy as np
import matplotlib.pyplot as plt
import math

# Start
start_north             = np.float64(0.0)
start_east              = np.float64(0.0)

mid_north               = np.float64(100.0)
mid_east                = np.float64(100.0)

end_north               = np.float64(0.0)
end_east                = np.float64(200.0)

segment_north           = [start_north, mid_north, end_north]
segment_east            = [start_east , mid_east , end_east ]
segment_idx             = 1                                     # First segment
max_segment_idx         = len(segment_north) - 1

######################## TEST_FUNCTION ########################

# Route segment component
def _get_route_segment_component(segment_idx):
    base_north      = segment_north[segment_idx - 1]
    base_east       = segment_east[segment_idx - 1]
    
    head_north      = segment_north[segment_idx]
    head_east       = segment_east[segment_idx]
    
    d_segment_north = head_north - base_north
    d_segment_east  = head_east  - base_east
    
    segment_length  = np.hypot(d_segment_north, d_segment_east)
    beta            = np.atan2(d_segment_east, d_segment_north)
    
    return base_north, base_east, head_north, head_east, segment_length, beta
    
# IW Sampler Component
def _get_iw_sampler_component(n_iw, e_iw, base_north, base_east, beta, segment_length):
    # Start to IW component lengths
    d_base_to_iw_north = n_iw - base_north
    d_base_to_iw_east  = e_iw - base_east
    
    # Start to IW length
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
    
def _scope_next_iw(scope_angle_deg, scope_length, beta, prev_wp_north, prev_wp_east):
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
# True Intermediate Waypoints List
n_iw_list = []
e_iw_list = []

# Ship trajectory
traj_n      = [start_north]
traj_e      = [start_east]

# IW Projection
p_n_list = []
p_e_list = []

# IW sampler components
traversed_segment_length_list   = []
untraversed_segment_length_list = []
segment_arm_length_list         = []

# Commands
# scope_angles_deg = [30, -30, -30]
# scope_angles_deg = [30, -30, -30, -15, 15]
scope_angles_deg = [30, -30, -30, -15, -30, 0, 15, 30, 0]
scope_length     = 25


# FIRST TRIGGER
# Set the current ship position as IW0
n_iw_0                  = n_ship
e_iw_0                  = e_ship

# Route component
base_north, base_east, head_north, head_east, segment_length, beta = _get_route_segment_component(segment_idx)

# Get the IW sampler component
(traversed_segment_length_0, 
untraversed_segment_length_0,
segment_arm_length_0, 
p_n_0, p_e_0) = _get_iw_sampler_component(n_iw_0, e_iw_0, base_north, base_east, beta, segment_length)

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
idx = 0
max_idx = len(scope_angles_deg)-1
segment_switch = False
loop_breaker = False
while idx <= max_idx:
    # Get scope angle
    scope_angle_deg = scope_angles_deg[idx]
    
    # Get segment info
    base_north, base_east, head_north, head_east, segment_length, beta = _get_route_segment_component(segment_idx)
    print("scope_angle_deg: ", scope_angle_deg)
    print("base_north: ", base_north)
    print("base_east: ", base_east)
    print("head_north: ", head_north)
    print("head_east: ", head_east)
    print("segment_length: ", segment_length)
    print("beta: ", np.rad2deg(beta))
    
    # Get the waypoint
    if segment_switch:
        print("THIS WILL SHOW UP ONLY ONCE AFTER THE SEGMENT SWITCH")
        prev_n_iw = base_north
        prev_e_iw = base_east
        segment_switch = False
    else:
        prev_n_iw = n_iw_list[idx]
        prev_e_iw = e_iw_list[idx]
    print("prev_n_iw: ", prev_n_iw)
    print("prev_e_iw: ", prev_e_iw)
    
    # Get the next waypoint
    n_iw, e_iw = _scope_next_iw(scope_angle_deg, scope_length, beta, prev_n_iw, prev_e_iw)
    print("n_iw: ", n_iw)
    print("e_iw: ", e_iw)
    
    # Get the IW sampler component
    (traversed_segment_length, 
    untraversed_segment_length,
    segment_arm_length, 
    p_n, p_e) = _get_iw_sampler_component(n_iw, e_iw, base_north, base_east, beta, segment_length)
    print("traversed_segment_length: ", traversed_segment_length)
    print("untraversed_segment_length: ", untraversed_segment_length)
    print("segment_arm_length: ", segment_arm_length)
    print("p_n: ", p_n)
    print("p_e: ", p_e)
    print("#####")
    
    # Condition to switch segment
    if untraversed_segment_length < scope_length:
        print("###################################################################")
        print("SEGMENT SWITCH")
        print("###################################################################")
        
        if segment_idx < max_segment_idx:
            segment_idx += 1
            segment_switch = True
            traj_n.append(head_north)
            traj_e.append(head_east)
        else:
            p_n_list.append(p_n)
            p_e_list.append(p_e)
            n_iw_list.append(n_iw)
            e_iw_list.append(e_iw)
            traj_n.append(n_iw)
            traj_e.append(e_iw)
            traj_n.append(head_north)
            traj_e.append(head_east)
            break
        continue
        
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
    
    # Increment the IW Sampler
    idx += 1
    
# Ensure trajectory is closed at the final route endpoint
final_north = segment_north[segment_idx]
final_east  = segment_east[segment_idx]

if traj_n[-1] != final_north or traj_e[-1] != final_east:
    traj_n.append(final_north)
    traj_e.append(final_east)
        
# Check
length_list = []
for i in range(len(e_iw_list)):
    if i >= (len(e_iw_list)-1):
        break
    length = np.hypot((e_iw_list[i+1]-e_iw_list[i]), (n_iw_list[i+1]-n_iw_list[i]))
    length_list.append(length)
    
print("length_list: ", length_list)
print("n_iw_list: ", n_iw_list)
print("e_iw_list: ", e_iw_list)
print("p_n_list: ", p_n_list)
print("p_e_list: ", p_e_list)
print("#####")

# ######################## PLOT ########################
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

plt.show()
