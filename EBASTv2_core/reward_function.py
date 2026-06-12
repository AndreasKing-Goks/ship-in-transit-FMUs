from EBASTv2_core.reward_designs import (RewardDesign1, RewardDesign2, RewardDesign3,
                                   RewardDesign4, RewardDesign5, RewardDesign6)

import numpy as np
from scipy.stats import truncnorm

# Instantiate Reward Design function
intercept_angle_reward_func     = RewardDesign1(target=0, offset_param=500)
iw_distance_to_os_reward_func   = RewardDesign2(target=1000, offset_param1=10000, offset_param2=10000000)
nearest_distance_reward_func    = RewardDesign4(target=100, offset_param=15000000)

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

    eps = 1e-12

    times = []

    # When A is close to zero, own ship velocity is close to the tar ship velocity
    if abs(A) < eps:
        # Linear case: B t + C = 0
        if abs(B) < eps:
            return None
        t = -C / B
        if t > 0:
            times.append(t)
    else:
        disc = B**2 - 4*A*C

        # If discriminant < 0, no feasible solution for t_collision
        if disc < 0:
            return None

        sqrt_disc = np.sqrt(disc)

        # All feasible solution
        t1 = (-B + sqrt_disc) / (2*A)
        t2 = (-B - sqrt_disc) / (2*A)

        for t in [t1, t2]:
            if t > 0:
                times.append(t)
    
    # If A and B is both close to zero, no feasible solution for t_collision
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
         reward_components, finish_intercept_flags,
         skip_map_evaluation,
         ts_iw_id, nearest_dist_dict,
         remaining_requests_bound,
         max_scope_angles, IW_coordinates,
         scope_angles, prev_scope_angles,
         previous_action_masks, routes_cog_ned_deg) = args
        
        # Initial reward signal
        reward      = 0.0

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

        ### Termination rewards
        if own_ship_collision:
            rew_osc = 5.0
            reward += rew_osc
            reward_components["own_ship_collision_rewards"].append(rew_osc)
        else:
            reward_components["own_ship_collision_rewards"].append(0.0)
            
        if own_ship_grounding:
            rew_osg = 5.0
            reward += rew_osg
            reward_components["own_ship_grounding_rewards"].append(rew_osg)
        else:
            reward_components["own_ship_grounding_rewards"].append(0.0)
            
        if own_ship_navigation_failure:
            rew_osn = 3.0
            reward += rew_osn
            reward_components["own_ship_navigational_failure_rewards"].append(rew_osn)
        else:
            reward_components["own_ship_navigational_failure_rewards"].append(0.0)
            
        if own_ship_reaches_end_waypoint:
            rew_osr = -2.0
            reward += rew_osr
            reward_components["own_ship_reaches_end_waypoint_rewards"].append(rew_osr)
        else:
            reward_components["own_ship_reaches_end_waypoint_rewards"].append(0.0)
            
        if tar_ships_collision:
            rew_tsc = -2.0
            reward += rew_tsc
            reward_components["tar_ships_collision_rewards"].append(rew_tsc)
        else:
            reward_components["tar_ships_collision_rewards"].append(0.0)
            
        if tar_ships_grounding:
            rew_tsg = -2.0
            reward += rew_tsg
            reward_components["tar_ships_grounding_rewards"].append(rew_tsg)
        else:
            reward_components["tar_ships_grounding_rewards"].append(0.0)
            
        if tar_ships_navigation_failure:
            rew_tsn = -1.0
            reward += rew_tsn
            reward_components["tar_ships_navigation_failure_rewards"].append(rew_tsn)
        else:
            reward_components["tar_ships_navigation_failure_rewards"].append(0.0)
            
        if tar_ships_reaches_end_waypoint:
            rew_tsr = -1.0
            reward += rew_tsr
            reward_components["tar_ships_reaches_end_waypoint_rewards"].append(rew_tsr)
        else:
            reward_components["tar_ships_reaches_end_waypoint_rewards"].append(0.0)
        
        ### Non-termination rewards
        # Unpack the observation:
        own_ship_pos        = observation["own_ship_pos"]
        own_ship_speed      = observation["own_ship_forward_speed"]
        rel_tar_ships_pos   = observation["rel_tar_ships_pos"].reshape(n_ts, 3)
        tar_ships_speed     = observation["tar_ships_forward_speed"]
        remaining_requests  = observation["remaining_requests"]
        action_masks        = observation["action_masks"]
        
        # Compute the used requests to max requests ratio
        max_remaining_requests  = remaining_requests_bound["max"]
        used_requests           = max_remaining_requests - remaining_requests
        used_to_max_ratio       = used_requests / max_remaining_requests
        
        # Unpack own ship position
        own_north       = own_ship_pos[0]
        own_east        = own_ship_pos[1]
        
        # Unpack nearest distance
        ss_dist_list    = list(nearest_dist_dict.values())
        ss_dist_list_iw = []    # For IW enabled target ship
        for ts_id in list(nearest_dist_dict.keys()):
            if ts_id in ts_iw_id:
                ss_dist_list_iw.append(nearest_dist_dict[ts_id])
        
        ##############################################################################################
        ## REWARD 1: TOTAL DISTANCE REWARD
        # Mean of the sum of Reward 1a, Reward 1b, and Reward 1c
        
        # REWARD 1a: Nearest distance reward during state-action transition [FOR IW & NON-IW SHIP]
        rew_nd_ss       = np.mean([nearest_distance_reward_func(dist) for dist in ss_dist_list])
        
        # Store REWARD 1a
        reward_components["nearest_distance_rewards"].append(rew_nd_ss)
        
        #--------------------------------------------------------------------------------------------#
        
        # REWARD 1b: Intermediate waypoint nearest distance to own ship [FOR IW & NON-IW SHIP]
        iw_dist_list        = []
        for IW_coordinate in IW_coordinates:
            iw_north        = IW_coordinate[0]
            iw_east         = IW_coordinate[1]
            
            d_north         = own_north - iw_north
            d_east          = own_east  - iw_east
            
            dist            = np.hypot(d_north, d_east)
            iw_dist_list.append(dist)
        rew_nd_iw      = np.mean([iw_distance_to_os_reward_func(dist) for dist in iw_dist_list])
        
        # Store REWARD 1b
        reward_components["nearest_distance_iw_rewards"].append(rew_nd_iw)
        
        #--------------------------------------------------------------------------------------------#
        
        # REWARD 1c: Target ship nearest distance to the own ship when ROA [FOR IW & NON-IW SHIP]
        roa_dist_list       = []
        for rel_tar_ship_pos in rel_tar_ships_pos:
            rel_tar_north       = rel_tar_ship_pos[0]
            rel_tar_east        = rel_tar_ship_pos[1]
            
            dist            = np.hypot(rel_tar_north, rel_tar_east)         
            roa_dist_list.append(dist)
        rew_nd_roa          = np.mean([nearest_distance_reward_func(dist) for dist in roa_dist_list])
        
        # Store REWARD 1c
        reward_components["nearest_distance_roa_rewards"].append(rew_nd_roa)
        
        #--------------------------------------------------------------------------------------------#
        
        # COMPUTE REWARD 1
        rew_dist            = (rew_nd_ss + rew_nd_iw + rew_nd_roa)/3
        reward             += rew_dist
        
        # Store REWARD 1
        reward_components["total_distance_rewards"].append(rew_dist)
        
        ##############################################################################################
        
        ## REWARD 2: Intermediate waypoint sampling penalty/reward [FOR IW SHIP]
        rew_iwp_coeff           = 1.5
        rew_iwp                 = -(np.mean(used_to_max_ratio * rew_iwp_coeff))
        reward                 += rew_iwp
        
        # Store REWARD 2
        reward_components["scope_angle_request_done_rewards"].append(rew_iwp)
        
        ##############################################################################################
        
        ## REWARD 3: Scope angle change log likelihood reward [FOR IW SHIP]
        # Initially low influence then gets dominant
        rews_scc                    = []
        mean_change                 = 0.0
        rew_scc_coeff_multiplier    = 1.0
        sigma                       = 5.0
        
        for sc, psc, am, msc, umr in zip(scope_angles, prev_scope_angles, previous_action_masks, max_scope_angles, used_to_max_ratio):
            # Skip it if the action is masked
            if bool(am) != True:
                continue
            
            # Compute the low bound likelihood
            ll_scc_min          = logprior_scope_angle_change(scope_angle_change=0.0,
                                                              mean_change=mean_change,
                                                              sigma=sigma,
                                                              theta=2*msc)
            # Compute the high bound likelihood
            ll_scc_max          = logprior_scope_angle_change(scope_angle_change=2*msc,
                                                              mean_change=mean_change,
                                                              sigma=sigma,
                                                              theta=2*msc)
            
            # Compute the scope angle change likelihood
            scope_angle_change  = sc - psc
            ll_scc              = logprior_scope_angle_change(scope_angle_change=scope_angle_change,
                                                              mean_change=mean_change,
                                                              sigma=sigma,
                                                              theta=2*msc)
            
            # Compute the dynamic reward coefficient
            rew_scc_coeff       = umr * rew_scc_coeff_multiplier
            
            # Reward is based on the normalized scope angle change likelihood
            rew                 = ((ll_scc - ll_scc_min) / (ll_scc_min - ll_scc_max)) * rew_scc_coeff
            
            rews_scc.append(rew)
        rew_scc                 = np.mean(rews_scc)
        reward                 += rew_scc
        
        # Store REWARD 3
        reward_components["scope_angle_change_log_likelihood_rewards"].append(rew_scc) 
        
        ##############################################################################################
        
        ## REWARD 4: Interception scope angle reward [FOR IW SHIP]
        # Initially high influence then mellows down
        rew_isa_coeff_multiplier    = 1.0
        passing_discount_factor     = 0.75
        rews_isa                    = []
        zipped_list                 =zip(ts_iw_idx, scope_angles, previous_action_masks, 
                                         routes_cog_ned_deg, used_to_max_ratio, 
                                         ss_dist_list_iw, finish_intercept_flags)
        for i, (idx, sc, am, r_cog, umr, ss_dist_iw, fi) in enumerate(zipped_list):
            # Skip it if the action is masked
            if bool(am) != True:
                continue
            
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

            
            if return_val is not None:
                # When collision is feasible
                _, interception_angle_deg, _, _ = return_val
                desired_angle_deg               = sc + r_cog
                
                # Compute the error between the desired and the interception angle
                error           = desired_angle_deg - interception_angle_deg
                
                # Compute the dynamic reward coefficient
                rew_isa_coeff   = (2.0 - umr) * rew_isa_coeff_multiplier
                
                # Compute reward
                rew_isa         = intercept_angle_reward_func(error) * rew_isa_coeff
                
            else:
                # Reward is the nearest distance during transition converted using nearest distance reward design.
                # However this only valid for one passing. Once passed, set the flag as True and no rewards is obtained
                if fi is False:
                    rew_isa         = nearest_distance_reward_func(ss_dist_iw) * passing_discount_factor
                    finish_intercept_flags[i] = True

                else:
                    rew_isa         = 0.0
            
            rews_isa.append(rew_isa)            
            
        rew_isa             = np.mean(rews_isa)
        reward             += rew_isa
        
        # Store REWARD 4
        reward_components["intercept_scope_angle_rewards"].append(rew_isa)
        
        return reward