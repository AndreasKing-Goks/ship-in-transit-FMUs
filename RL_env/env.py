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
from orchestrator.scenario_config import load_base_config, prepare_config_and_spawn_requests_with_traffic_gen

class EBASTv2Env(gym.Env):
    """
        This class is the main class for Event-Based Adaptive Stress Testing v2 (EB-ASTv2)-compliant 
        for RL environment wrapper for gymnasium library.
    """
    def __init__(self,
                 ROOT,
                 config_path,
                 encounter_settings_path,
                 anim_save_path):
        # Store necessary path
        self.ROOT                       = ROOT
        self.config_path                = config_path
        self.encounter_settings_path    = encounter_settings_path
        self.anim_save_path             = anim_save_path
        
        # Save the base configuration for the Ship in Transit Co-simulation
        config_base         = load_base_config(config_path)
        self.simu_config    = config_base["simulation"]
        self.ship_configs   = config_base["ships"]
        
        # Count all of the ship assets
        self.n_s            = len(self.ship_configs)
        self.n_ts           = self.n_ship - 1           # Minus the own ship
        
        # List and count target ships that enables IW sampling
        self.ts_iw_id       = []
        self.ts_iw_idx      = []
        for idx, ship_config in enumerate(self.ship_configs):
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
        
        # Reset the environment
        self.reset()
            
    
    def _init_action_space(self):
        """
            Action space is defined as a vector of scope angles, where the dimension 
            of the vector is defined by the amount intermediate waypoint sampling-
            allowed target ships. Scope angle will be used to compute the next 
            intermediate waypoints
        """
        # List all of the maximum scope angle for each target ship mission manager
        self.scope_angles_bound = {}
        for sid, idx in zip(self.ts_iw_id, self.ts_iw_idx):
            max_scope_angle = self.ship_configs[idx]["fmu_params"]["MISSION_MANAGER"]["scope_angle_max_deg"]
            
            data = {
                "min": -max_scope_angle,
                "max": max_scope_angle
            }
            
            self.scope_angles_bound[sid] = data
        
        # Define the action space (normalized)
        self.action_space = Box(
            low  = np.array([-1.0]*self.n_ts_iw, dtype=np.float32),
            high = np.array([1.0]*self.n_ts_iw, dtype=np.float32)
        )
        
        
    def _init_observation_space(self):
        """
            Observation space is defined as collection of exposed states:
                - COLAV states: list(float)
                    [throttle, new_rudder_angle]
                - Own ship states: list(float)
                    [north, east, yaw_angle, forward_speed, cross_track_error]
                - Target ship(s) states: list(float)
                    [north, east, yaw_angle, forward_speed] * n_ts
                - Action masks: list(float(binary))
                    [action_mask] * n_ts   
                - Remaining requests: list(float(round val, mimic int))
                    [remaining_requests] * n_ts
        """
        ## Observation space bounds [min, max]
        # Collision avoidance states
        self.throttle_bound         = [0.0, 1.0]
        max_rudder_angle_deg        = self.ship_configs[0]["fmu_params"]["AUTOPILOT"]["max_rudder_angle_deg"]
        self.new_rudder_angle_bound = [-max_rudder_angle_deg, max_rudder_angle_deg]
        
        # Own ship states
        self.OS0_north_bound            = [-20000, 20000]                           # Arbitrary
        self.OS0_east_bound             = [-20000, 20000]                           # Arbitrary
        self.OS0_yaw_angle_deg_bound    = [-np.pi, np.pi]                           # Follow NED angle
        self.OS0_e_ct_bound             = [-500, 500]                               # Arbitrary
        os_sogMax                       = self.ship_configs[0].get("sogMax", 13.0)  # If not specified, assume max speed of ground is 13 m/s
        self.OS0_forward_speed_bound    = [0, os_sogMax]
        
        # Target ship states
        for ts_id in range(self.n_ts):
            setattr(self, f"TS{ts_id+1}_north_bound", [-20000, 20000])
            setattr(self, f"TS{ts_id+1}_east_bound", [-20000, 20000])
            setattr(self, f"TS{ts_id+1}_yaw_angle_deg_bound", [-np.pi, np.pi])
            ts_sogMax = self.ship_configs[ts_id+1].get("sogMax", 13.0)  # If not specified, assume max speed of ground is 13 m/s
            setattr(self, f"TS{ts_id+1}_forward_speed_bound", [0, ts_sogMax])
            
        # Action masks (For all target ships)
        self.action_masks_bound         = [0.0, 1.0]
        
        # Remaining requests (For all target ships)
        max_requests = []
        for ship_config in self.ship_configs:
            id = ship_config.get("id")
            
            if id not in self.ts_iw_id:
                max_requests.append(0)
            else:
                max_count = ship_config["fmu_params"]["MISSION_MANAGER"]["max_sampled_inter_wp"]
                max_requests.append(max_count)
        self.remaining_requests         = [0.0, max(max_requests)]
            
        # Define the observation space (normalized)
        self.observation_space = gym.spaces.Dict(
            {
                "colav_states": Box(-1.0, 1.0, shape=(2,), dtype=np.float32),
                "own_ship_pos": Box(-1.0, 1.0, shape=(3,), dtype=np.float32),
                "own_ship_ect": Box(-1.0, 1.0, shape=(1,), dtype=np.float32),
                "own_ship_vel": Box(0.0, 1.0, shape=(1,), dtype=np.float32),
                "ts_ship_pos": Box(-1.0, 1.0, shape=(3*self.n_ts,), dtype=np.float32),
                "ts_ship_vel": Box(0.0, 1.0, shape=(1*self.n_ts,), dtype=np.float32),
                "action_masks": Box(0.0, 1.0, shape=(1*self.n_ts), dtype=np.float32),
                "remaining_requests": Box(0.0, 1.0, shape=(1*self.n_ts), dtype=np.float32)
            }
        )
    
        
    def _normalize(self, x, min_val, max_val):
        """Normalize x from [min_val, max_val] to [-1, 1]."""
        return 2 * (x - min_val) / (max_val - min_val) - 1
    
    
    def _denormalize(self, x_norm, min_val, max_val):
        """Denormalize x from [-1, 1] back to [min_val, max_val]."""
        return (x_norm + 1) * 0.5 * (max_val - min_val) + min_val
    
    
    def _normalize_action(self, action):
        action_norm_list = []
        
        n_action    = self.action_space.shape[0]
        action_low  = self.action_space.low
        action_high = self.action_space.high
        
        for i in range(n_action):
            action_norm_list.append(self._normalize_action(action[i], action_low[i], action_high[i]))
            
        action_norm      = tuple(action_norm_list)
        
        return action_norm      
    
    def _denormalize_action(self, action_norm):
        action_list = []
        
        n_action    = self.action_space.shape[0]
        action_low  = self.action_space.low
        action_high = self.action_space.high
        
        for i in range(n_action):
            action_list.append(self._normalize_action(action_norm[i], action_low[i], action_high[i]))
            
        action      = tuple(action_list)
        
        return action      
    
    
    def _normalize_observation(self, observation):
        pass
    
    
    def _denormalize_observation(self, observation_norm):
        pass
    
    
    def _compute_reward(self, observation):
        """
            Compute reward for taken action that transition agent from current state to next state
            to fulfill Markov Decisison Process.
        """
        reward = 0
        
        # Own ship collision
        
        # Non own ship collision
        
        # Own ship mission finished
        
        # Target ship mission finished
        
        # Own ship grounding
        
        # Target ship grounding
        
        # Own ship navigation failure
        
        # Target ship navigation failure
        
        # Intermediate waypoint sampling count
        
        # Trajectory smoothness
        
        return reward
    
    
    def step(self, action):
        pass
    
    
    def reset(self, seed: Optional[int] = None, options: Optional[dict] = None):
        """
            Reset the environment all together
        """
        
        # IMPORTAT: Seed the random number generator
        super().reset(seed=seed)
        self.np_random, _ = gym.utils.seeding.np_random(seed)
        
        # Generate ship traffic generator's spawn requests
        availableNavStatus = [
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
        own_ship_initial = {
            "position": {
                "north": 0.0,
                "east": 0.0,
            },
            "sog": 10.0,    # m/s
            "cog": 0.0,
            "heading": 0.0,
            "navStatus": "Under way using engine",
        }
        
        if seed:
            random.seed(seed=seed)
        
        encounter_type_TS1 = random.choice(availableEncounterTypes)
        encounter_type_TS2 = random.choice(availableEncounterTypes)
        
        encounters = {
            "TS1" : {
                "desiredEncounterType": encounter_type_TS1,
                "vectorTime": 10.0,
                "beta": -5.0,
                "relativeSpeed": 1.2,
                },
            "TS2" : {
                "desiredEncounterType": encounter_type_TS2,
                "vectorTime": 15.0,
                "beta": -60.0,
                "relativeSpeed": 1.2,
                },
        }
        
        config, spawn_requests = prepare_config_and_spawn_requests_with_traffic_gen(own_ship_initial, 
                                                                                    encounters, 
                                                                                    self.config_path, 
                                                                                    self.encounter_settings_path)
        
        # Instantiate Co-simulation wrapper
        self.instance = ShipInTransitCoSimulation(config=config, 
                                                  spawn_requests=spawn_requests, 
                                                  ROOT=self.ROOT)