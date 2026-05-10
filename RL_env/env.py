""" 
This module provides classes for EB-ASTv2-compliant environment wrapper
"""
from typing import Optional

import gymnasium as gym
from gymnasium.spaces import Box

import numpy as np
import pandas as pd
import random

from orchestrator.sit_cosim import ShipInTransitCoSimulation
from orchestrator.scenario_config import (prepare_config_and_spawn_requests_with_traffic_gen,
                                          sample_beta_and_rel_speed_given_encounter_settings,
                                          load_base_config)

from RL_env.reward_designs import (RewardDesign1, RewardDesign2, RewardDesign3,
                                   RewardDesign4, RewardDesign5, RewardDesign6)

class EBASTv2Env(gym.Env):
    """
        This class is the main class for Event-Based Adaptive Stress Testing v2 (EB-ASTv2)-compliant 
        for RL environment wrapper for gymnasium library.
        
        WARNING!
        Always do reset() before running the environment!
    """
    def __init__(self,
                 ROOT,
                 config_path,
                 encounter_settings_path,
                 skip_map_evaluation    : bool=True):
        # Map evaluation flag
        self.skip_map_evaluation        = skip_map_evaluation
        
        # Store necessary path
        self.ROOT                       = ROOT
        self.config_path                = config_path
        self.encounter_settings_path    = encounter_settings_path
        
        # Save the base configuration for the Ship in Transit Co-simulation
        config_base         = load_base_config(config_path)
        self.simu_config    = config_base["simulation"]
        self.ship_configs   = config_base["ships"]
        
        # Count all of the ship assets
        self.n_s            = len(self.ship_configs)
        self.n_ts           = self.n_s - 1           # Minus the own ship
        
        # Compile own ship ID and target ship IDs
        self.ts_id          = []
        
        # List and count target ships that enables IW sampling
        self.ts_iw_id       = []
        self.ts_iw_idx      = []
        for idx, ship_config in enumerate(self.ship_configs):
            if idx==0:
                self.os_id = (ship_config["id"])
            else:
                self.ts_id.append(ship_config["id"])
            
            IW_sampling_cfg = ship_config.get("IW_sampling", None)
            
            has_IW_sampling = False
            if isinstance(IW_sampling_cfg, dict):
                has_IW_sampling = True
                
            if has_IW_sampling:
                self.ts_iw_id.append(ship_config["id"])
                self.ts_iw_idx.append(idx)
        
        self.n_ts_iw        = len(self.ts_iw_id)
        
        # Initialize action space
        self._init_action_space()
        
        # Initialize observation space
        self._init_observation_space()
        
        # Initialize the reward design
        self._init_reward_design()
            
    
    def _init_action_space(self):
        """
            Action space is defined as a vector of scope angles and scope lengths, 
            where the dimension  of the vector is defined by the amount intermediate 
            waypoint sampling-allowed target ships. Scope angle and scope length will 
            be used to compute the next intermediate waypoints
        """
        # List all of the maximum scope angle for each target ship mission manager
        self.action_bound = {}
        
        # List all the low and high bound of the action
        action_low = []
        action_high = []
        
        for sid, idx in zip(self.ts_iw_id, self.ts_iw_idx):
            max_scope_angle     = self.ship_configs[idx]["fmu_params"]["MISSION_MANAGER"]["scope_angle_max_deg"]
            min_scope_angle     = -max_scope_angle
            max_scope_length    = 2500
            min_scope_length    = 1500
            
            data = {
                "min": [min_scope_angle, min_scope_length],
                "max": [max_scope_angle, max_scope_length]
            }
            
            self.action_bound[sid] = data
            
            # Flattened bounds
            action_low.extend([min_scope_angle, min_scope_length])
            action_high.extend([max_scope_angle, max_scope_length])
        
        self.action_low = np.array(action_low, dtype=np.float32)
        self.action_high = np.array(action_high, dtype=np.float32)
        
        # normalized RL action space
        self.action_space = Box(
            low=-1.0,
            high=1.0,
            shape=(len(action_low),),
            dtype=np.float32
        )
        
        
    def _init_observation_space(self):
        """
            Observation space is defined as collection of exposed states:
                - COLAV states: list(float)
                    [new_throttle, new_rudder_angle]
                - Own ship states: list(float)
                    [north, east, yaw_angle, cross_track_error, rudder_angle, throttle, forward_speed]
                - Target ship(s) states: list(float)
                    [north, east, yaw_angle, forward_speed] * n_ts
                - Action masks: list(float(binary))
                    [action_mask] * n_ts   
                - Remaining requests: list(float(round val, mimic int))
                    [remaining_requests] * n_ts
        """
        
        # Positional states range
        north_bound             = [-30000, 30000]                                   # Arbitrary
        east_bound              = [-30000, 30000]                                   # Arbitrary
        yaw_angle_deg_bound     = [-np.pi, np.pi]                                   # Follow NED angle
        
        pos_min_bound           = [north_bound[0], east_bound[0], yaw_angle_deg_bound[0]]
        pos_max_bound           = [north_bound[1], east_bound[1], yaw_angle_deg_bound[1]]
        
        # Forward speed range
        sogMax_list             = []
        for ship_config in self.ship_configs:
            sogMax = ship_config.get("sogMax", 13.0)  # If not specified, assume max speed of ground is 13 m/s
            sogMax_list.append(sogMax)
        sogMax = max(sogMax_list)
        
        ## Observation space bounds [min, max]
        # Collision avoidance states bounds
        new_throttle_bound      = [0.0, 1.0]
        max_rudder_angle_deg    = self.ship_configs[0]["fmu_params"]["AUTOPILOT"]["max_rudder_angle_deg"]
        new_rudder_angle_bound  = [-max_rudder_angle_deg, max_rudder_angle_deg]
        self.colav_states_bound = {
            "min": np.array([new_throttle_bound[0], new_rudder_angle_bound[0]], dtype=np.float32),
            "max": np.array([new_throttle_bound[1], new_rudder_angle_bound[1]], dtype=np.float32)
        }
        
        ## Own ship bounds
        self.own_ship_pos_bound = {
            "min": np.array(pos_min_bound, dtype=np.float32),
            "max": np.array(pos_max_bound, dtype=np.float32)
        }
        
        self.own_ship_e_ct_bound            = {
            "min": np.array([-500.0], dtype=np.float32),
            "max": np.array([500], dtype=np.float32)
        }
        
        self.own_ship_rudder_angle_bound    = {
            "min": np.array([-max_rudder_angle_deg], dtype=np.float32),
            "max": np.array([max_rudder_angle_deg], dtype=np.float32)
        }
        
        self.own_ship_throttle_bound        = {
            "min": np.array([0.0], dtype=np.float32),
            "max": np.array([1.0], dtype=np.float32)
        }
        
        self.own_ship_forward_speed_bound   = {
            "min": np.array([0.0], dtype=np.float32),
            "max": np.array([sogMax], dtype=np.float32)
        }
        
        # Target ships bounds
        self.tar_ships_pos_bound = {
            "min": np.array(pos_min_bound*self.n_ts, dtype=np.float32),
            "max": np.array(pos_max_bound*self.n_ts, dtype=np.float32)
        }
        
        self.tar_ships_forward_speed_bound = {
            "min": np.array([0.0]*self.n_ts, dtype=np.float32),
            "max": np.array([sogMax]*self.n_ts, dtype=np.float32)
        }
        
        # Action masks (For all target ships with IW sampler enabled)
        self.action_masks_bound             = {
            "min": np.array([0.0]*self.n_ts_iw, dtype=np.float32),
            "max": np.array([1.0]*self.n_ts_iw, dtype=np.float32)
        }
        
        # Remaining requests (For all target ships with IW sampler enabled)
        max_requests = []
        for ship_config in self.ship_configs:
            id = ship_config.get("id")
            
            if id not in self.ts_iw_id:
                continue
            else:
                max_count = ship_config["fmu_params"]["MISSION_MANAGER"]["max_sampled_inter_wp"]
                max_requests.append(max_count)
        min_requests = [0.0] * len(max_requests)
        self.remaining_requests_bound = {
            "min": np.array(min_requests, dtype=np.float32),
            "max": np.array(max_requests, dtype=np.float32)
        }
            
        # Define the observation space (normalized)
        self.observation_space = gym.spaces.Dict(
            {
                "colav_states": Box(-1.0, 1.0, shape=(2,), dtype=np.float32),
                "own_ship_pos": Box(-1.0, 1.0, shape=(3,), dtype=np.float32),
                "own_ship_e_ct": Box(-1.0, 1.0, shape=(1,), dtype=np.float32),
                "own_ship_rud_ang": Box(-1.0, 1.0, shape=(1,), dtype=np.float32),
                "own_ship_throttle": Box(-1.0, 1.0, shape=(1,), dtype=np.float32),
                "own_ship_forward_speed": Box(-1.0, 1.0, shape=(1,), dtype=np.float32),

                "tar_ships_pos": Box(-1.0, 1.0, shape=(3*self.n_ts,), dtype=np.float32),
                "tar_ships_forward_speed": Box(-1.0, 1.0, shape=(self.n_ts,), dtype=np.float32),

                # Binary mask for IW-enabled target ships only
                "action_masks": gym.spaces.MultiBinary(self.n_ts_iw),

                "remaining_requests": Box(-1.0, 1.0, shape=(self.n_ts_iw,), dtype=np.float32),
            }
        )
    
    
    def _init_reward_design(self):
        """
            Design a non-linear function for non-termination case reward functions
            Output reward is between 0 and 1
        """
        self.nearest_distance_reward = RewardDesign4(target=100, offset_param=750000)
    
    
    def _normalize(self, x, x_min, x_max):
        """
        Maps [x_min, x_max] -> [-1, 1]
        """
        x = np.asarray(x, dtype=np.float32)
        x_min = np.asarray(x_min, dtype=np.float32)
        x_max = np.asarray(x_max, dtype=np.float32)

        return 2.0 * (x - x_min) / (x_max - x_min) - 1.0


    def _denormalize(self, x_norm, x_min, x_max):
        """
        Maps [-1, 1] -> [x_min, x_max]
        """
        x_norm = np.asarray(x_norm, dtype=np.float32)
        x_min = np.asarray(x_min, dtype=np.float32)
        x_max = np.asarray(x_max, dtype=np.float32)

        return 0.5 * (x_norm + 1.0) * (x_max - x_min) + x_min
    
    
    def _normalize_action(self, action):
        action = np.asarray(action, dtype=np.float32)
        action_norm = self._normalize(
            action,
            self.action_low,
            self.action_high
        )
        action_norm = self._safe_clip(action_norm, -1.0, 1.0)
        return action_norm   
    
    
    def _denormalize_action(self, action_norm):
        action_norm = np.asarray(action_norm, dtype=np.float32)
        action = self._denormalize(
            action_norm,
            self.action_low,
            self.action_high
        )
        return action  
    
    def _normalize_observation(self, observation):
        """
            Normalized the actual observation to [-1,1]
        """
        observation_norm = {
            "colav_states": self._normalize(observation["colav_states"], self.colav_states_bound["min"], self.colav_states_bound["max"]),
            "own_ship_pos": self._normalize(observation["own_ship_pos"], self.own_ship_pos_bound["min"], self.own_ship_pos_bound["max"]),
            "own_ship_e_ct": self._normalize(observation["own_ship_e_ct"], self.own_ship_e_ct_bound["min"], self.own_ship_e_ct_bound["max"]),
            "own_ship_rud_ang": self._normalize(observation["own_ship_rud_ang"], self.own_ship_rudder_angle_bound["min"], self.own_ship_rudder_angle_bound["max"]),
            "own_ship_throttle": self._normalize(observation["own_ship_throttle"], self.own_ship_throttle_bound["min"], self.own_ship_throttle_bound["max"]),
            "own_ship_forward_speed": self._normalize(observation["own_ship_forward_speed"], self.own_ship_forward_speed_bound["min"], self.own_ship_forward_speed_bound["max"]),
            "tar_ships_pos": self._normalize(observation["tar_ships_pos"], self.tar_ships_pos_bound["min"], self.tar_ships_pos_bound["max"]),
            "tar_ships_forward_speed": self._normalize(observation["tar_ships_forward_speed"], self.tar_ships_forward_speed_bound["min"], self.tar_ships_forward_speed_bound["max"]),

            "action_masks": np.asarray(observation["action_masks"], dtype=np.int8),

            "remaining_requests": self._normalize(observation["remaining_requests"], self.remaining_requests_bound["min"], self.remaining_requests_bound["max"]),
        }

        return observation_norm
    
    def _denormalize_observation(self, observation_norm):
        """
            Denormalized the normalized observation to its actual value
        """
        observation = {
            "colav_states": self._denormalize(observation_norm["colav_states"], self.colav_states_bound["min"], self.colav_states_bound["max"]),
            "own_ship_pos": self._denormalize(observation_norm["own_ship_pos"], self.own_ship_pos_bound["min"], self.own_ship_pos_bound["max"]),
            "own_ship_e_ct": self._denormalize(observation_norm["own_ship_e_ct"], self.own_ship_e_ct_bound["min"], self.own_ship_e_ct_bound["max"]),
            "own_ship_rud_ang": self._denormalize(observation_norm["own_ship_rud_ang"], self.own_ship_rudder_angle_bound["min"], self.own_ship_rudder_angle_bound["max"]),
            "own_ship_throttle": self._denormalize(observation_norm["own_ship_throttle"], self.own_ship_throttle_bound["min"], self.own_ship_throttle_bound["max"]),
            "own_ship_forward_speed": self._denormalize(observation_norm["own_ship_forward_speed"], self.own_ship_forward_speed_bound["min"], self.own_ship_forward_speed_bound["max"]),
            "tar_ships_pos": self._denormalize(observation_norm["tar_ships_pos"], self.tar_ships_pos_bound["min"], self.tar_ships_pos_bound["max"]),
            "tar_ships_forward_speed": self._denormalize(observation_norm["tar_ships_forward_speed"], self.tar_ships_forward_speed_bound["min"], self.tar_ships_forward_speed_bound["max"]),

            "action_masks": np.asarray(observation_norm["action_masks"], dtype=np.int8),

            "remaining_requests": self._denormalize(observation_norm["remaining_requests"], self.remaining_requests_bound["min"], self.remaining_requests_bound["max"]),
        }

        return observation
    
    
    def _safe_clip(self, x, low=-1.0, high=1.0):
        # Prevent observation to go out of declared space's bound
        x = np.asarray(x, dtype=np.float32)
        x = np.nan_to_num(x, nan=0.0, posinf=high, neginf=low)
        return np.clip(x, low, high).astype(np.float32)
    
    
    def _get_obs(self, normalized=True):
        """
            Get the RL observations
        """
        ## Get the actual values
        own_ship_id         = self.ship_configs[0]["id"]
        
        # Colav states
        new_throttle        = self.instance.GetLastValue(slaveName=f"{own_ship_id}__COLAV",
                                                         slaveVar="new_throttle_cmd")
        new_rudder_angle    = self.instance.GetLastValue(slaveName=f"{own_ship_id}__COLAV",
                                                         slaveVar="new_rudder_angle_deg")
        
        # Own ship 
        own_north           = self.instance.GetLastValue(slaveName=f"{own_ship_id}__SHIP_MODEL",
                                                         slaveVar="north")
        own_east            = self.instance.GetLastValue(slaveName=f"{own_ship_id}__SHIP_MODEL",
                                                         slaveVar="east")
        own_heading         = self.instance.GetLastValue(slaveName=f"{own_ship_id}__SHIP_MODEL",
                                                         slaveVar="yaw_angle_rad")
        own_e_ct            = self.instance.GetLastValue(slaveName=f"{own_ship_id}__AUTOPILOT",
                                                         slaveVar="e_ct")
        own_rud_ang         = self.instance.GetLastValue(slaveName=f"{own_ship_id}__AUTOPILOT",
                                                         slaveVar="rudder_angle_deg")
        own_throttle        = self.instance.GetLastValue(slaveName=f"{own_ship_id}__THROTTLE_CONTROLLER",
                                                         slaveVar="throttle_cmd")
        own_forward_speed   = self.instance.GetLastValue(slaveName=f"{own_ship_id}__SHIP_MODEL",
                                                         slaveVar="forward_speed")        
        
        # Target ship
        tar_ship_pos_list           = []
        tar_ship_forward_speed_list = []
        action_masks_list           = []
        remaining_requests_list     = []
        
        for ship_config in self.instance.ship_configs:
            # Get target ship id
            tar_ship_id = ship_config["id"]
            
            # Skip if ship id belongs to the own ship
            if tar_ship_id == own_ship_id:
                continue
            
            # Target Ship
            tar_north           = self.instance.GetLastValue(slaveName=f"{tar_ship_id}__SHIP_MODEL",
                                                         slaveVar="north")
            tar_east            = self.instance.GetLastValue(slaveName=f"{tar_ship_id}__SHIP_MODEL",
                                                            slaveVar="east")
            tar_heading         = self.instance.GetLastValue(slaveName=f"{tar_ship_id}__SHIP_MODEL",
                                                            slaveVar="yaw_angle_rad")
            tar_forward_speed   = self.instance.GetLastValue(slaveName=f"{tar_ship_id}__SHIP_MODEL",
                                                         slaveVar="forward_speed")        
            
            # Compile the lists
            tar_ship_pos        = [tar_north, tar_east, tar_heading]
            tar_ship_pos_list.extend(tar_ship_pos)
            tar_ship_forward_speed_list.append(tar_forward_speed)
            
            # For target ship that has intermediate waypoint sampling enabled
            if tar_ship_id in self.ts_iw_id:
                request_captain_intent  = self.instance.GetLastValue(slaveName=f"{tar_ship_id}__MISSION_MANAGER",
                                                                     slaveVar="request_captain_intent")
                max_sampled_inter_wp    = self.instance.GetLastValue(slaveName=f"{tar_ship_id}__MISSION_MANAGER",
                                                                     slaveVar="max_sampled_inter_wp")
                acc_sampled_inter_wp    = self.instance.GetLastValue(slaveName=f"{tar_ship_id}__MISSION_MANAGER",
                                                                     slaveVar="accepted_sampled_inter_wp")
                remaining_request       = max_sampled_inter_wp - acc_sampled_inter_wp
                
                # Append IW sampler-related observations
                action_masks_list.append(int(request_captain_intent))
                remaining_requests_list.append(remaining_request)
        
        # Convert to numpy array
        colav_states                 = np.array([new_throttle, new_rudder_angle], dtype=np.float32)
        own_ship_pos                 = np.array([own_north, own_east, own_heading], dtype=np.float32)
        own_ship_e_ct                = np.array([own_e_ct], dtype=np.float32)
        own_ship_rud_ang             = np.array([own_rud_ang], dtype=np.float32)
        own_ship_throttle            = np.array([own_throttle], dtype=np.float32)
        own_ship_forward_speed       = np.array([own_forward_speed], dtype=np.float32)
        tar_ships_pos                = np.array(tar_ship_pos_list, dtype=np.float32)
        tar_ships_forward_speed      = np.array(tar_ship_forward_speed_list, dtype=np.float32)
        action_masks                 = np.array(action_masks_list, dtype=np.int8)                   # <- Action space is MultiBinary
        remaining_requests           = np.array(remaining_requests_list, dtype=np.float32)
        
        # Get observation
        observation = {
                "colav_states": colav_states,
                "own_ship_pos": own_ship_pos,
                "own_ship_e_ct": own_ship_e_ct,
                "own_ship_rud_ang": own_ship_rud_ang,
                "own_ship_throttle": own_ship_throttle,
                "own_ship_forward_speed": own_ship_forward_speed,
                "tar_ships_pos": tar_ships_pos,
                "tar_ships_forward_speed": tar_ships_forward_speed,
                "action_masks": action_masks,
                "remaining_requests": remaining_requests
            }
        
        # Get normalized observation when required
        if normalized:
            return self._normalize_observation(observation)
        else:
            return observation
        
    
    def _get_info(self, reset_attempts= None, action_masks=None, ts_iw_id_masked=None, action_masked=None):
        # Empty info container
        info = {}
        
        # Info for regular env step method
        if action_masks is not None and ts_iw_id_masked is not None and action_masked is not None:
            info["action_masks"]        = action_masks.copy()
            info["applied_ship_ids"]    = list(ts_iw_id_masked)
            info["applied_actions"]     = action_masked.copy()
        
        # Info for reset method
        elif reset_attempts is not None:
            info["reset_attempts"]      = reset_attempts
            
        return info
    
    
    def _compute_reward(self, observation):
        """
            Compute reward after the environment transitions to the next state.
            Observation needs to be denormalized first
        """
        reward      = 0.0

        stop_info   = self.instance.stop_info

        ### Own ship events
        own_ship_info                   = stop_info[self.os_id]

        own_ship_collision              = own_ship_info["collision"]["status"][-1]
        own_ship_grounding              = own_ship_info["grounding"][-1] if not self.skip_map_evaluation else False
        own_ship_navigation_failure     = own_ship_info["navigation_failure"][-1]
        own_ship_reaches_end_waypoint   = own_ship_info["reaches_end_waypoint"][-1]

        ### Target ship events
        tar_ships_collision             = False
        tar_ships_grounding             = False
        tar_ships_navigation_failure    = False
        tar_ships_reaches_end_waypoint  = False

        for ts_id in self.ts_id:
            ts_info = stop_info[ts_id]

            tar_ship_collision = ts_info["collision"]["status"][-1]
            tar_ship_colliders = ts_info["collision"]["colliders"][-1]

            # Bad target-ship collision: target ship collides with anything except own ship
            if tar_ship_collision and (self.os_id not in tar_ship_colliders):
                tar_ships_collision = True

            if not self.skip_map_evaluation:
                if ts_info["grounding"][-1]:
                    tar_ships_grounding = True

            if ts_info["navigation_failure"][-1]:
                tar_ships_navigation_failure = True

            if ts_info["reaches_end_waypoint"][-1]:
                tar_ships_reaches_end_waypoint = True

        ### Termination rewards
        if own_ship_collision:
            reward += 5
        if own_ship_grounding:
            reward += 5
        if own_ship_navigation_failure:
            reward += 3
        if own_ship_reaches_end_waypoint:
            reward += -5
        if tar_ships_collision:
            reward += -5
        if tar_ships_grounding:
            reward += -5
        if tar_ships_navigation_failure:
            reward += -5
        if tar_ships_reaches_end_waypoint:
            reward += -1
        
        ### Non-termination rewards
        ## Unpack the observation:
        own_ship_pos        = observation["own_ship_pos"]
        tar_ships_pos       = observation["tar_ships_pos"].reshape(self.n_ts, 3)
        remaining_requests  = observation["remaining_requests"]
        
        ## Nearest distance reward
        own_north       = own_ship_pos[0]
        own_east        = own_ship_pos[1]
        
        # Compute the distance of each target ship to the own ship
        dist_list = []
        for unshifted_idx in self.ts_iw_idx:
            idx             = unshifted_idx - 1
            
            tar_ship_pos    = tar_ships_pos[idx]
            
            tar_north       = tar_ship_pos[0]
            tar_east        = tar_ship_pos[1]
            
            d_north         = own_north - tar_north
            d_east          = own_east - tar_east
            
            dist            = np.hypot(d_north, d_east)         
            dist_list.append(dist)
        
        # Get the minimum distance
        min_dist    = min(dist_list)      
        
        # Compute the nearest distance reward
        reward      += self.nearest_distance_reward(min_dist)
        
        ## Intermediate waypoint sampling penalty/reward
        iws_count_coeff         = 0.05
        max_remaining_requests  = self.remaining_requests_bound["max"]
        used_requests           = max_remaining_requests - remaining_requests
        used_to_max_ratio       = used_requests / max_remaining_requests
        reward                 += -np.sum(used_to_max_ratio) * iws_count_coeff
        
        # Trajectory smoothness penalty/reward
        # TBD

        return reward
    
    
    def _get_config_and_spawn_requests(self):
        # Generate ship traffic generator's spawn requests
        availableNavStatus      = [
            "Under way using engine",
            "At anchor",
            "Not under command",
            "Restricted maneuverability",
            "Constrained by draft",
            "Moored",
            "Aground",
            "Engaged in fishing",
            "Under way sailing",
            "Reserved for future use"
        ]
        availableEncounterTypes = [
            "head-on", 
            "overtaking-give-way",
            "overtaking-stand-on",
            "crossing-give-way",
            "crossing-stand-on"
        ]
        vectorTime              = [15, 20, 25]
        
        own_ship_initial        = {
            "position": {
                "north": 0.0,
                "east": 0.0,
            },
            "sog": 10.0,    # m/s
            "cog": 0.0,
            "heading": 0.0,
            "navStatus": "Under way using engine",
        }
        
        # Sample encounters
        encounters = {}
        for ship_config in self.ship_configs:
            # Ship ID
            ship_id = ship_config["id"]
            
            # Sample encounter type, then sample relative bearing and relative speed based on it
            encounter_type = self.np_random.choice(availableEncounterTypes)
            beta, rel_speed = sample_beta_and_rel_speed_given_encounter_settings(encounter_type,
                                                                                 self.encounter_settings_path,
                                                                                 rng=self.np_random)
            vector_time     = self.np_random.choice(vectorTime)
            
            encounter = {
                "desiredEncounterType": encounter_type,
                "vectorTime": vector_time,
                "beta": beta,
                "relativeSpeed": rel_speed,
            }
            
            encounters[ship_id] = encounter
        
        # Get the config and spawn requests based on the Ship Traffic Generator
        config, spawn_requests = prepare_config_and_spawn_requests_with_traffic_gen(own_ship_initial, 
                                                                                    encounters, 
                                                                                    self.config_path, 
                                                                                    self.encounter_settings_path)
        return config, spawn_requests
    
    
    def _step(self, action_dict=None):
        """
            The method is used for stepping up the Co-simulation.
            Action is a dictionary of scope angles and scope length 
            and the target ship that requests captain_intent
        """
        # If action_dict exists, manipulate the FMU
        # Else, skip the action_dict
        if action_dict is not None:
            for ship_id, action in list(action_dict.items()):
                # Unpack the action and manipulate the FMUs
                scope_angle     = action["scope_angle"]
                scope_length    = action["scope_length"]

                self.instance.SingleVariableManipulation(
                    slaveName=f"{ship_id}__MISSION_MANAGER",
                    slaveVar="scope_angle_deg",
                    value=scope_angle
                )
                self.instance.SingleVariableManipulation(
                    slaveName=f"{ship_id}__MISSION_MANAGER",
                    slaveVar="scope_length",
                    value=scope_length
                )
                
        # Step up the simulator
        self.instance.step()
    
    
    def _advance_until_next_event(self, action_dict=None):
        """
            Do simulator step until and event is found
        """
        any_request = False

        while not any_request:
            # Simulator integration
            self._step(action_dict)
            
            # Set action dict as None once used to enable 
            # simulator step up without action
            if isinstance(action_dict, dict):
                action_dict = None

            # Check if the simulator still within the maximum simulation time
            if self.instance.time > self.instance.stopTime:
                self.truncated = True
                return False

            # Check if all the ship assets has stopped
            if not self.instance.stop:
                self.instance.time += self.instance.stepSize
            else:
                self.terminated = True
                return False

            # If not truncated or terminated, check the request captain intent for all of the enabled ships
            request_list = []
            for ts_iw_id in self.ts_iw_id:
                request = self.instance.GetLastValue(
                    slaveName=f"{ts_iw_id}__MISSION_MANAGER",
                    slaveVar="request_captain_intent"
                )
                request_list.append(bool(request))

            # If one or more enabled target ships request captain intent
            any_request = any(request_list)

        return True


    def step(self, action_norm):
        """
           The method is used to step up the Reinforcement Learning step. 
        """
        # Get the action masks
        observation     = self._get_obs()
        action_masks    = observation["action_masks"].astype(bool)
        
        # Get the array version of action and IW sampler enabled target ship
        action = self._denormalize_action(action_norm).reshape(self.n_ts_iw, 2)
        
        # Mask the action and the target ship ID
        action_masked   = action[action_masks]
        ts_iw_id_masked = np.array(self.ts_iw_id)[action_masks]
        
        # Compile the action dictionary for _step()
        action_dict     = {}
        for act, ts_iw_id in zip(action_masked, ts_iw_id_masked):
            action_dict[ts_iw_id] = {
                "scope_angle": act[0],
                "scope_length": act[1]
            }
        
        # Advance the simulation until an event is found
        self._advance_until_next_event(action_dict=action_dict)
                 
        # Get observation and info
        observation             = self._get_obs()                                           # This is normalized
        reward                  = self._compute_reward(self._get_obs(normalized=False))     # Use denormalized observation
        terminated              = self.terminated
        truncated               = self.truncated
        info                    = self._get_info(
                                    action_masks=action_masks,
                                    ts_iw_id_masked=ts_iw_id_masked,
                                    action_masked=action_masked
                                    )

        return observation, reward, terminated, truncated, info
    
    
    def reset(self, seed: Optional[int] = None, options: Optional[dict] = None):
        """
            Reset the environment all together
        """
        # IMPORTANT: Seed the random number generator
        super().reset(seed=seed)
        self.np_random, _ = gym.utils.seeding.np_random(seed)
        
        # Do repeated attempts of reset until an event happened
        max_attempts = 20

        for attempt in range(max_attempts):            
            # Run the simulator until at least one of the enabled ship request captaint intent
            self.terminated         = False
            self.truncated          = False
            
            # Use Ship Traffic Generator to generates collision encounter case
            config, spawn_requests = self._get_config_and_spawn_requests()

            # Instantiate the ShipInTransitCoSimulation
            self.instance = ShipInTransitCoSimulation(
                config=config,
                spawn_requests=spawn_requests,
                ROOT=self.ROOT,
                skip_map_evaluation=self.skip_map_evaluation
            )
            
            # List for recording one episode
            self.obs_list = []
            self.action_list = []
            self.action_time_list = []
            self.reward_list = []
            self.terminated_list = []
            self.truncated_list = []
            self.info_list = []
            
            # Advance the simulation until an event is found
            found_event = self._advance_until_next_event()

            # If an event is encountered, output observation and info as a valid
            # reset attempt
            if found_event:
                observation = self._get_obs()
                info = self._get_info(reset_attempts=attempt + 1)
                return observation, info

        raise RuntimeError(
            "Failed to initialize an episode with a captain-intent request "
            f"after {max_attempts} attempts."
        )