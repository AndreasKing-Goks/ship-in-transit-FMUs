from RL_env.reward_designs import (RewardDesign1, RewardDesign2, RewardDesign3,
                                   RewardDesign4, RewardDesign5, RewardDesign6)

import numpy as np

# Instantiate Reward Function 4 for nearest distance reward function
nearest_distance_reward_func = RewardDesign4(target=100, offset_param=750000)

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
         ts_iw_idx, nearest_dist_dict,
         remaining_requests_bound) = args
        
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
            rew     = 2
            reward += rew
            reward_components["own_ship_collision_rewards"].append(rew)
        else:
            reward_components["own_ship_collision_rewards"].append(0.0)
            
        if own_ship_grounding:
            rew     = 2
            reward += rew
            reward_components["own_ship_grounding_rewards"].append(rew)
        else:
            reward_components["own_ship_grounding_rewards"].append(0.0)
            
        if own_ship_navigation_failure:
            rew     = 1
            reward += rew
            reward_components["own_ship_navigational_failure_rewards"].append(rew)
        else:
            reward_components["own_ship_navigational_failure_rewards"].append(0.0)
            
        if own_ship_reaches_end_waypoint:
            rew     = -2
            reward += rew
            reward_components["own_ship_reaches_end_waypoint_rewards"].append(rew)
        else:
            reward_components["own_ship_reaches_end_waypoint_rewards"].append(0.0)
            
        if tar_ships_collision:
            rew     = -2
            reward += rew
            reward_components["tar_ships_collision_rewards"].append(rew)
        else:
            reward_components["tar_ships_collision_rewards"].append(0.0)
            
        if tar_ships_grounding:
            rew     = -2
            reward += rew
            reward_components["tar_ships_grounding_rewards"].append(rew)
        else:
            reward_components["tar_ships_grounding_rewards"].append(0.0)
            
        if tar_ships_navigation_failure:
            rew     = -1
            reward += rew
            reward_components["tar_ships_navigation_failure_rewards"].append(rew)
        else:
            reward_components["tar_ships_navigation_failure_rewards"].append(0.0)
            
        if tar_ships_reaches_end_waypoint:
            rew     = -1
            reward += rew
            reward_components["tar_ships_reaches_end_waypoint_rewards"].append(rew)
        else:
            reward_components["tar_ships_reaches_end_waypoint_rewards"].append(0.0)
        
        ### Non-termination rewards
        ## Unpack the observation:
        own_ship_pos        = observation["own_ship_pos"]
        tar_ships_pos       = observation["tar_ships_pos"].reshape(n_ts, 3)
        remaining_requests  = observation["remaining_requests"]
        
        ## Nearest distance reward when RoA
        own_north       = own_ship_pos[0]
        own_east        = own_ship_pos[1]
        
        # Compute the distance of each target ship to the own ship when RoA
        dist_list = []
        for unshifted_idx in ts_iw_idx:
            idx             = unshifted_idx - 1
            
            tar_ship_pos    = tar_ships_pos[idx]
            
            tar_north       = tar_ship_pos[0]
            tar_east        = tar_ship_pos[1]
            
            d_north         = own_north - tar_north
            d_east          = own_east - tar_east
            
            dist            = np.hypot(d_north, d_east)         
            dist_list.append(dist)
        
        # Get the nearest distance when RoA
        rew         = np.mean([nearest_distance_reward_func(dist) for dist in dist_list])
        reward     += rew
        reward_components["nearest_distance_roa_rewards"].append(rew)
        
        ## Nearest distance reward during state-action transition
        dist_list   = list(nearest_dist_dict.values())
        rew         = np.mean([nearest_distance_reward_func(dist) for dist in dist_list])
        reward     += rew
        reward_components["nearest_distance_rewards"].append(rew)
        
        ## Intermediate waypoint sampling penalty/reward
        iws_count_coeff         = 0.05
        max_remaining_requests  = remaining_requests_bound["max"]
        used_requests           = max_remaining_requests - remaining_requests
        used_to_max_ratio       = used_requests / max_remaining_requests
        rew                     = -np.sum(used_to_max_ratio) * iws_count_coeff
        reward                 += rew
        reward_components["scope_angle_request_done_rewards"].append(rew)
        
        # Scope angle likelihood reward
        # TBD

        return reward