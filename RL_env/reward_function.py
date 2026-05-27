from RL_env.reward_designs import (RewardDesign1, RewardDesign2, RewardDesign3,
                                   RewardDesign4, RewardDesign5, RewardDesign6)

import numpy as np
from scipy.stats import truncnorm

# Instantiate Reward Function 4 for nearest distance reward function
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
    
def compute_reward(observation, args):
        """
            Compute reward after the environment transitions to the next state.
            Observation needs to be denormalized first
        """
        # Unpack args
        (os_id, ts_id, 
         stop_info, n_ts,
         reward_components,
         skip_map_evaluation,
         n_ts_iw, nearest_dist_dict,
         remaining_requests_bound,
         max_scope_angles, IW_coordinates,
         scope_angles, prev_scope_angles,
         previous_action_masks) = args
        
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
        ## Unpack the observation:
        own_ship_pos        = observation["own_ship_pos"]
        rel_tar_ships_pos   = observation["rel_tar_ships_pos"].reshape(n_ts, 3)
        remaining_requests  = observation["remaining_requests"]
        
        ## Nearest distance reward when RoA
        own_north       = own_ship_pos[0]
        own_east        = own_ship_pos[1]
        
        ## Nearest distance reward during state-action transition
        ss_dist_list    = list(nearest_dist_dict.values())
        rew_nd_ss       = np.mean([nearest_distance_reward_func(dist) for dist in ss_dist_list])
        reward         += rew_nd_ss
        reward_components["nearest_distance_rewards"].append(rew_nd_ss)
        
        ## Intermediate waypoint nearest distance to own ship
        iw_dist_list        = []
        for IW_coordinate in IW_coordinates:
            iw_north        = IW_coordinate[0]
            iw_east         = IW_coordinate[1]
            
            d_north         = own_north - iw_north
            d_east          = own_east  - iw_east
            
            dist            = np.hypot(d_north, d_east)
            iw_dist_list.append(dist)
            
        # Get the IW distance to own_ship
        rew_nd_iw      = np.mean([iw_distance_to_os_reward_func(dist) for dist in iw_dist_list])
        reward          += rew_nd_iw
        reward_components["nearest_distance_iw_rewards"].append(rew_nd_iw)
        
        # Compute the distance of each target ship to the own ship when RoA
        roa_dist_list       = []
        for rel_tar_ship_pos in rel_tar_ships_pos:
            rel_tar_north       = rel_tar_ship_pos[0]
            rel_tar_east        = rel_tar_ship_pos[1]
            
            dist            = np.hypot(rel_tar_north, rel_tar_east)         
            roa_dist_list.append(dist)
        
        # Get the nearest distance when RoA
        rew_nd_roa          = np.mean([nearest_distance_reward_func(dist) for dist in roa_dist_list])
        reward             += rew_nd_roa
        reward_components["nearest_distance_roa_rewards"].append(rew_nd_roa)
        
        ## Intermediate waypoint sampling penalty/reward
        iws_count_coeff         = 1.0
        max_remaining_requests  = remaining_requests_bound["max"]
        used_requests           = max_remaining_requests - remaining_requests
        used_to_max_ratio       = used_requests / max_remaining_requests
        rew_iwp                 = -(np.sum(used_to_max_ratio) * iws_count_coeff)
        reward                 += rew_iwp
        reward_components["scope_angle_request_done_rewards"].append(rew_iwp)
        
        # Scope angle change log likelihood reward
        rews_scc                = []
        mean_change             = 0.0
        rew_coeff               = n_ts_iw        # Linearly dependent to the amount of ts_iw
        sigma                   = 5.0
        
        # Next time should include the masks here
        for sc, psc, am, msc in zip(scope_angles, prev_scope_angles, previous_action_masks, max_scope_angles):
            # Skip it if the action is masked
            if bool(am) != True:
                continue
            
            ll_scc_min              = logprior_scope_angle_change(scope_angle_change=0.0,
                                                              mean_change=mean_change,
                                                              sigma=sigma,
                                                              theta=2*msc)
            ll_scc_max              = logprior_scope_angle_change(scope_angle_change=2*msc,
                                                              mean_change=mean_change,
                                                              sigma=sigma,
                                                              theta=2*msc)
            
            scope_angle_change  = sc - psc
            ll_scc              = logprior_scope_angle_change(scope_angle_change=scope_angle_change,
                                                              mean_change=mean_change,
                                                              sigma=sigma,
                                                              theta=2*msc)
            
            rew                 = ((ll_scc - ll_scc_min) / (ll_scc_min - ll_scc_max)) * rew_coeff
            
            rews_scc.append(rew)
        rew_scc                 = np.mean(rews_scc)
        reward                 += rew_scc
        reward_components["scope_angle_change_log_likelihood_rewards"].append(rew_scc) 

        return reward