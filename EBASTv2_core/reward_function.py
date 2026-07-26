from EBASTv2_core.reward_designs import (RewardDesign1, RewardDesign2, RewardDesign3,
                                         RewardDesign4, RewardDesign5, RewardDesign6,
                                         RewardDesign7)

import numpy as np
from scipy.stats import truncnorm

# Instantiate Reward Design function
intercept_angle_reward_func     = RewardDesign1(target=0, offset_param=500)
nearest_distance_reward_func    = RewardDesign7(collision_zone_radius=100.0) # Match with ship_config["fmu_params"]["COLAV"].get("collision_zone_radius")

def wrap_angle(x):
    # wrap to (-pi, pi]
    return (x + np.pi) % (2*np.pi) - np.pi

def logprior_scope_angle_change(
    scope_angle_change,
    mean_change,
    sigma=5.0,
    theta=60.0
):
    """
        Truncated normal distribution prior for the sampled scope angle change 
        around the mean 0. The idea is that RL agent would ideally keep  it 
        course heading the same as the previous scope angle it used. The larger
        the change of sampled scope angle to the previously sampled scope angle,
        the more unlikely it is
        
        (a, b) is a measure of how many standard deviations away the lower and
        upper bounds from the mean value.

        Support:
            [-theta, theta] degrees
    """
    
    # Upper and Lower bound
    lower = -theta
    upper = theta

    # Normalized
    a = (lower - mean_change) / sigma
    b = (upper - mean_change) / sigma

    return truncnorm.logpdf(
        scope_angle_change,
        a,
        b,
        loc=mean_change,
        scale=sigma
    )
    
def find_collision_heading(pos_own, vel_own, pos_tar, speed_tar):
    """
    Find target heading that would make target collide with own ship.
    Recorded when first enter RoA

    pos_own   : [north, east]
    vel_own   : [v_north, v_east]
    pos_tar   : [north, east]
    speed_tar : scalar target speed magnitude

    Returns
    -------
    heading_rad : heading angle in radians, measured from north-east coordinate using atan2(east, north)
    heading_deg : heading angle in degrees
    t_collision : collision time
    direction   : unit direction vector [north, east]
    """

    p_o = np.asarray(pos_own, dtype=float)
    v_o = np.asarray(vel_own, dtype=float)
    p_t = np.asarray(pos_tar, dtype=float)

    # Relative position from target to own ship
    r = p_o - p_t

    # Quadratic coefficients
    A = np.dot(v_o, v_o) - speed_tar**2
    B = 2 * np.dot(r, v_o)
    C = np.dot(r, r)
    
    print("A    : ", A)
    print("B    : ", B)
    print("C    : ", C)

    eps = 1e-12

    times = []

    # When A is close to zero, own ship velocity is close to the tar ship velocity
    if abs(A) < eps:
        # Linear case: B t + C = 0
        if abs(B) < eps:
            return None
        t = -C / B
        print("t    : ", t)
        if t > eps:
            times.append(t)
    else:
        disc = B**2 - 4*A*C
        print("disc : ", disc)

        # If discriminant < 0, no feasible solution for t_collision
        if disc < 0:
            return None

        sqrt_disc = np.sqrt(disc)

        # All feasible solution
        t1 = (-B + sqrt_disc) / (2*A)
        t2 = (-B - sqrt_disc) / (2*A)
        
        print("t1   : ", t1)
        print("t2   : ", t2)

        # Safe guard for realistic collision times
        max_collision_time = 7200.0  # example: 120 minutes
        
        for t in [t1, t2]:
            if t > 0 and t <= max_collision_time:
                times.append(t)
    
    # No positive root means no feasible solution for t_collision
    if not times:
        return None

    # Usually choose the earliest future collision
    t_collision = min(times)

    # Required target velocity vector
    vel_tar_required = (p_o + v_o * t_collision - p_t) / t_collision

    # Unit heading direction
    direction = vel_tar_required / np.linalg.norm(vel_tar_required)

    # Heading angle: atan2(east, north)
    heading_rad = np.arctan2(direction[1], direction[0])
    heading_deg = np.degrees(heading_rad)

    return heading_rad, heading_deg, t_collision, direction
    
def compute_reward(observation, args):
    """
        Compute reward after the environment transitions to the next state.
        Observation needs to be denormalized first
    """
    # Unpack args
    (os_id, ts_id, ts_iw_idx,
        stop_info, n_ts,
        reward_components,
        skip_map_evaluation,
        ts_iw_id, nearest_dist_dict,
        remaining_requests_bound,
        max_scope_angles, detailed_rewards,
        scope_angles, prev_scope_angles,
        previous_action_masks, routes_cog_ned_deg) = args

    ### Own ship events
    own_ship_info                   = stop_info[os_id]

    own_ship_collision              = own_ship_info["collision"]["status"][-1]
    own_ship_grounding              = own_ship_info["grounding"][-1] if not skip_map_evaluation else False
    own_ship_navigation_failure     = own_ship_info["navigation_failure"][-1]
    own_ship_reaches_end_waypoint   = own_ship_info["reaches_end_waypoint"][-1]

    ### Target ship events
    tar_ships_collision             = False
    tar_ships_grounding             = False
    tar_ships_navigation_failure    = False
    tar_ships_reaches_end_waypoint  = False

    for ts_id in ts_id:
        ts_info = stop_info[ts_id]

        tar_ship_collision = ts_info["collision"]["status"][-1]
        tar_ship_colliders = ts_info["collision"]["colliders"][-1]

        # Bad target-ship collision: target ship collides with anything except own ship
        if tar_ship_collision and (os_id not in tar_ship_colliders):
            tar_ships_collision = True

        if not skip_map_evaluation:
            if ts_info["grounding"][-1]:
                tar_ships_grounding = True

        if ts_info["navigation_failure"][-1]:
            tar_ships_navigation_failure = True

        if ts_info["reaches_end_waypoint"][-1]:
            tar_ships_reaches_end_waypoint = True
            
    # Unpack the observation:
    own_ship_pos        = observation["own_ship_pos"]
    own_ship_speed      = observation["own_ship_forward_speed"]
    rel_tar_ships_pos   = observation["rel_tar_ships_pos"].reshape(n_ts, 3)
    tar_ships_speed     = observation["tar_ships_forward_speed"]
    remaining_requests  = observation["remaining_requests"]
    # action_masks        = observation["action_masks"]
    
    # Compute the used requests to max requests ratio
    max_remaining_requests  = remaining_requests_bound["max"]
    ratio_increment         = 1 / max_remaining_requests
    used_requests           = max_remaining_requests - remaining_requests
    used_to_max_ratio       = used_requests / max_remaining_requests
    
    # Unpack target ship-own ship nearest distance list
    nearest_dist_list_iw = []    # For IW enabled target ship
    for ts_id in list(nearest_dist_dict.keys()):
        if ts_id in ts_iw_id:
            nearest_dist_list_iw.append(nearest_dist_dict[ts_id])
    
    # Compute target ship-own ship final distance list
    final_dist_list       = []
    for rel_tar_ship_pos in rel_tar_ships_pos:
        rel_tar_north       = rel_tar_ship_pos[0]
        rel_tar_east        = rel_tar_ship_pos[1]
        
        dist            = np.hypot(rel_tar_north, rel_tar_east)         
        final_dist_list.append(dist)
            
    ##############################################################################################
    # TERMINATION REWARDS
    ##############################################################################################
    # Initial termination reward value:
    termination_reward      = None
    
    #---------------------------------------------------------------------------------------------
    # POSITIVE TERMINATION
    if own_ship_collision or own_ship_grounding or own_ship_navigation_failure:
        termination_reward  = 0.0
    
    #---------------------------------------------------------------------------------------------
    # NEGATIVE TERMINATION
    rew_fd_coeff_multiplier = 1.0
    
    if (own_ship_reaches_end_waypoint or tar_ships_collision or tar_ships_grounding
        or tar_ships_navigation_failure or tar_ships_reaches_end_waypoint):
        
        # Reward for final distance between target ships and own ships
        final_distance_reward   = np.mean([nearest_distance_reward_func(dist) for dist in  final_dist_list])
                
        # Compute the reward for negative termination
        termination_reward      = rew_fd_coeff_multiplier * final_distance_reward
        
        if detailed_rewards:
            reward_components["final_distance_reward"] = final_distance_reward
    
    #---------------------------------------------------------------------------------------------
    # Store termination reward
    reward_components["termination_reward"].append(termination_reward) 
    
    ##############################################################################################
    # NON-TERMINATION REWARDS
    ##############################################################################################
    # Initial non-termination reward value:
    non_termination_reward = 0.0
    
    #---------------------------------------------------------------------------------------------
    # REWARD 1: Scope angle change log likelihood reward [FOR IW SHIP]
    # Initially not too punishing then gets dominant
    rews_sac                    = []
    mean_change                 = 0.0
    rew_sac_coeff_multiplier    = 1.0
    sigma                       = 5.0
    
    for sc, psc, am, msc, umr in zip(scope_angles, prev_scope_angles, previous_action_masks, max_scope_angles, used_to_max_ratio):
        # Skip it if the action is masked
        if bool(am) != True:
            continue
        
        # Compute the low bound likelihood
        ll_sac_min          = logprior_scope_angle_change(scope_angle_change=0.0,
                                                            mean_change=mean_change,
                                                            sigma=sigma,
                                                            theta=2*msc)
        # Compute the high bound likelihood
        ll_sac_max          = logprior_scope_angle_change(scope_angle_change=2*msc,
                                                            mean_change=mean_change,
                                                            sigma=sigma,
                                                            theta=2*msc)
        
        # Compute the scope angle change likelihood
        scope_angle_change  = sc - psc
        ll_sac              = logprior_scope_angle_change(scope_angle_change=scope_angle_change,
                                                            mean_change=mean_change,
                                                            sigma=sigma,
                                                            theta=2*msc)
        
        # Compute the dynamic reward coefficient
        rew_sac_coeff       = umr * rew_sac_coeff_multiplier
        
        # Reward is based on the normalized scope angle change likelihood
        rew_sac             = ((ll_sac - ll_sac_min) / (ll_sac_min - ll_sac_max)) * rew_sac_coeff
        
        print("rew_sac : ", rew_sac)
        
        rews_sac.append(rew_sac)
        
    scope_angle_change_log_likelihood_rewards   = np.mean(rews_sac)
    non_termination_reward                     += scope_angle_change_log_likelihood_rewards
    
    # Store REWARD 1
    if detailed_rewards:
        reward_components["scope_angle_change_log_likelihood_rewards"].append(scope_angle_change_log_likelihood_rewards) 
    
    #---------------------------------------------------------------------------------------------
    # REWARD 2: Interception scope angle reward [FOR IW SHIP]
    # Initially high rewarding then mellows down
    rew_isa_coeff_multiplier    = 1.0
    passing_factor              = 0.5
    rews_isa                    = []
    zipped_list                 =zip(ts_iw_idx, scope_angles, previous_action_masks, 
                                        routes_cog_ned_deg, used_to_max_ratio, 
                                        ratio_increment, nearest_dist_list_iw)
    
    for i, (idx, sc, am, r_cog, umr, ri, n_dist_iw) in enumerate(zipped_list):
        # Skip it if the action is masked
        if bool(am) != True:
            continue
        
        print(f"TS {idx}")
        print(f"list index {i}")
        
        ts_idx              = idx - 1
        tar_ship_pos        = rel_tar_ships_pos[ts_idx] + own_ship_pos
        tar_ship_speed      = tar_ships_speed[ts_idx]
        
        # Prepare args for find_collision_heading function
        pos_own             = own_ship_pos[:2]
        pos_tar             = tar_ship_pos[:2]
         
        yaw_own             = own_ship_pos[2]
        
        vel_own             = own_ship_speed * np.array([np.cos(yaw_own), np.sin(yaw_own)])
        speed_tar           = tar_ship_speed
        
        return_val          = find_collision_heading(pos_own, vel_own, pos_tar, speed_tar)
        
        print("return_val :", return_val)

        if return_val is not None:
            # When collision is feasible
            _, interception_angle_deg, _, _ = return_val
            desired_angle_deg               = sc + r_cog
            
            # Compute the error between the desired and the interception angle
            error           = wrap_angle(desired_angle_deg - interception_angle_deg)
            
            # Compute the dynamic reward coefficient
            rew_isa_coeff   = ((1.0 + ri) - umr) * rew_isa_coeff_multiplier
            
            # Compute reward
            rew_isa         = (intercept_angle_reward_func(error) - 1) * rew_isa_coeff
            
        else:
            # Reward is the nearest distance during transition converted using nearest distance reward design
            # when no feasible collision in forseeable. Meaning we still negatively reward the agent when collision
            # is not found, however the reward is based on how near the last nearest encounter was. (Near encounter,
            # closer reward zero. Far encounter, really negative reward)
            
            rew_isa         = nearest_distance_reward_func(n_dist_iw) * passing_factor

            print("rew_isa : ", rew_isa)
            
        rews_isa.append(rew_isa)
        
    intercept_scope_angle_rewards   = np.mean(rews_isa)
    non_termination_reward         += intercept_scope_angle_rewards
    
    # Store REWARD 2
    if detailed_rewards:
        reward_components["intercept_scope_angle_rewards"].append(intercept_scope_angle_rewards)
    
    #---------------------------------------------------------------------------------------------
    # Store non-termination reward
    reward_components["non_termination_reward"].append(non_termination_reward) 
    
    # Reward per event
    if termination_reward is not None:
        reward = termination_reward + non_termination_reward
    else:
        reward = non_termination_reward
    
    return reward