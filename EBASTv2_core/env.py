""" 
This module provides classes for EB-ASTv2-compliant environment wrapper
"""
from pathlib import Path

from typing import Optional

import gymnasium as gym
from gymnasium.spaces import Box

import numpy as np
import pandas as pd

from orchestrator.scenario_config import load_base_config

from EBASTv2_core.reward_function import compute_reward

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
                 spawn_requests_bank,
                 skip_map_evaluation: bool=True,
                 custom_pos_bound: dict=None,
                 use_fmpy: bool=False):
        # Decide which orchestrator to use
        # If FMPy
        if use_fmpy:
            from orchestrator.sit_cosim_fmpy import ShipInTransitCoSimulation
        # If Libcosimpy
        else:
            from orchestrator.sit_cosim import ShipInTransitCoSimulation
        self.use_fmpy                   = use_fmpy
        self.orchestrator               = ShipInTransitCoSimulation
        
        # Set up placeholder attributes for the simulator instance
        self.instance                   = None
        
        # Initially set the environment for training
        self.for_training               = True
        
        # Map evaluation flag
        self.skip_map_evaluation        = skip_map_evaluation
        
        # Store necessary path
        self.ROOT                       = ROOT
        self.config_path                = config_path
        self.encounter_settings_path    = encounter_settings_path
        
        # Store the spawn requests to generate finite encounter cases
        self.spawn_requests_bank        = spawn_requests_bank
        self.n_spawn_cases              = self.spawn_requests_bank["n_cases"]
        self.start_eval_case_id         = self.spawn_requests_bank["start_eval_case_id"]
        self.spawn_cases                = self.spawn_requests_bank["cases"]
        self.srb_rel_path               = Path(self.spawn_requests_bank["path"])
        
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
        self._init_observation_space(custom_pos_bound)
        
    
    def set_for_training(self):
        self.for_training   = True
        
    
    def set_for_evaluation(self):
        self.for_training   = False
    
    
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
        
        # Collection of max_scope_angle for all target ships
        self.max_scope_angles   = []
        
        for sid, idx in zip(self.ts_iw_id, self.ts_iw_idx):
            max_scope_angle     = self.ship_configs[idx]["fmu_params"]["MISSION_MANAGER"]["scope_angle_max_deg"]
            self.max_scope_angles.append(max_scope_angle)
            min_scope_angle     = -max_scope_angle
            max_scope_length    = 7500
            min_scope_length    = 1000
            
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
            shape=(2*self.n_ts_iw,),
            dtype=np.float32
        )
        
        
    def _init_observation_space(self, custom_pos_bound:dict=None):
        """
            Observation space is defined as collection of exposed states:
                - COLAV states: list(float)
                    [new_throttle, new_rudder_angle]
                - Own ship states: list(float)
                    [north, east, yaw_angle, cross_track_error, rudder_angle, throttle, forward_speed]
                - Target ship(s) states: list(float)
                    [north, east, yaw_angle, forward_speed] * n_ts
                - Action masks: list(float)
                    [action_mask] * n_ts   
                - Remaining requests: list(float(round val, mimic int))
                    [remaining_requests] * n_ts
        """
        
        # Positional states range
        if custom_pos_bound is not None:
            rel_north_bound         = custom_pos_bound["north_bound"]                   # Arbitrary
            rel_east_bound          = custom_pos_bound["east_bound"]                    # Arbitrary
        else:
            rel_north_bound         = self.spawn_requests_bank["rounded_north_bound"]
            rel_east_bound          = self.spawn_requests_bank["rounded_east_bound"]
        yaw_angle_deg_bound     = [-np.pi, np.pi]                                       # Follow NED angle
        
        pos_min_bound           = [rel_north_bound[0], rel_east_bound[0], yaw_angle_deg_bound[0]]
        pos_max_bound           = [rel_north_bound[1], rel_east_bound[1], yaw_angle_deg_bound[1]]
        
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

                "rel_tar_ships_pos": Box(-1.0, 1.0, shape=(3*self.n_ts,), dtype=np.float32),
                "tar_ships_forward_speed": Box(-1.0, 1.0, shape=(self.n_ts,), dtype=np.float32),

                "action_masks": Box(low=0.0, high=1.0, shape=(self.n_ts_iw,), dtype=np.float32),

                "remaining_requests": Box(-1.0, 1.0, shape=(self.n_ts_iw,), dtype=np.float32),
            }
        )
    
    
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

        assert action.shape == self.action_low.shape, \
            f"Expected action shape {self.action_low.shape}, got {action.shape}"

        action_norm = self._normalize(
            action,
            self.action_low,
            self.action_high
        )

        return self._safe_clip(action_norm, -1.0, 1.0)
    
    
    def _denormalize_action(self, action_norm):
        action_norm = np.asarray(action_norm, dtype=np.float32)

        assert action_norm.shape == self.action_low.shape, \
            f"Expected action_norm shape {self.action_low.shape}, got {action_norm.shape}"

        action_norm = self._safe_clip(action_norm, -1.0, 1.0)

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
            "rel_tar_ships_pos": self._normalize(observation["rel_tar_ships_pos"], self.tar_ships_pos_bound["min"], self.tar_ships_pos_bound["max"]),
            "tar_ships_forward_speed": self._normalize(observation["tar_ships_forward_speed"], self.tar_ships_forward_speed_bound["min"], self.tar_ships_forward_speed_bound["max"]),

            "action_masks": (np.asarray(observation["action_masks"], dtype=np.float32) > 0.5).astype(np.float32),

            "remaining_requests": self._normalize(np.asarray(observation["remaining_requests"], dtype=np.float32),
                                                  self.remaining_requests_bound["min"],
                                                  self.remaining_requests_bound["max"]).astype(np.float32),
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
            "rel_tar_ships_pos": self._denormalize(observation_norm["rel_tar_ships_pos"], self.tar_ships_pos_bound["min"], self.tar_ships_pos_bound["max"]),
            "tar_ships_forward_speed": self._denormalize(observation_norm["tar_ships_forward_speed"], self.tar_ships_forward_speed_bound["min"], self.tar_ships_forward_speed_bound["max"]),

            "action_masks": (np.asarray(observation_norm["action_masks"], dtype=np.float32) > 0.5).astype(np.float32),

            "remaining_requests": np.rint(self._denormalize(observation_norm["remaining_requests"],
                                                            self.remaining_requests_bound["min"],
                                                            self.remaining_requests_bound["max"])).astype(np.float32),
        }
        return observation
    
    
    def _safe_clip(self, x, low=-1.0, high=1.0):
        # Prevent observation to go out of declared space's bound
        x = np.asarray(x, dtype=np.float32)
        x = np.nan_to_num(x, nan=0.0, posinf=high, neginf=low)
        return np.clip(x, low, high).astype(np.float32)
    
    def _wrap_angle(self, x):
        # wrap to (-pi, pi]
        return (x + np.pi) % (2*np.pi) - np.pi
    
    
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
            
            rel_tar_north       = tar_north - own_north
            rel_tar_east        = tar_east - own_east
            
            # Compile the lists
            tar_ship_pos        = [rel_tar_north, rel_tar_east, tar_heading]
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
        rel_tar_ships_pos            = np.array(tar_ship_pos_list, dtype=np.float32)
        tar_ships_forward_speed      = np.array(tar_ship_forward_speed_list, dtype=np.float32)
        
        action_masks                 = np.asarray(action_masks_list, dtype=np.float32)
        action_masks                 = (action_masks > 0.5).astype(np.float32)
        
        remaining_requests           = np.asarray(remaining_requests_list, dtype=np.float32)
        remaining_requests           = np.rint(remaining_requests).astype(np.float32)
        
        # Get observation
        observation = {
                "colav_states": colav_states,
                "own_ship_pos": own_ship_pos,
                "own_ship_e_ct": own_ship_e_ct,
                "own_ship_rud_ang": own_ship_rud_ang,
                "own_ship_throttle": own_ship_throttle,
                "own_ship_forward_speed": own_ship_forward_speed,
                "rel_tar_ships_pos": rel_tar_ships_pos,                         # Relative to own ship
                "tar_ships_forward_speed": tar_ships_forward_speed,
                "action_masks": action_masks,
                "remaining_requests": remaining_requests
            }
        
        # Get normalized observation when required
        if normalized:
            return self._normalize_observation(observation)
        else:
            return observation
        
    
    def _get_info(self, reset_attempts= None, case_idx=None, action_masks=None, ts_iw_id_masked=None, action_masked=None):
        # Empty info container
        info = {}
        
        # Info for regular env step method
        if action_masks is not None and ts_iw_id_masked is not None and action_masked is not None:
            info["action_masks"]        = action_masks.copy()
            info["applied_ship_ids"]    = list(ts_iw_id_masked)
            info["applied_actions"]     = action_masked.copy()
        
        # Info for reset method
        if reset_attempts is not None:
            info["reset_attempts"]      = reset_attempts
            
        # Info for case index selection
        if case_idx is not None:
            info["case_idx"] = case_idx
        
        return info
    
    
    def _get_distances_to_own_ship(self):
        # IDs
        os_id       = self.os_id
        ts_ids      = self.ts_id
        
        # Container
        dist_dict   = {}
        
        # Own ship position
        own_north   = self.instance.GetLastValue(slaveName=f"{os_id}__SHIP_MODEL",
                                                         slaveVar="north")
        own_east    = self.instance.GetLastValue(slaveName=f"{os_id}__SHIP_MODEL",
                                                         slaveVar="east")
        
        # For each target ships
        for ts_id in ts_ids:
            # Target ship position
            tar_north           = self.instance.GetLastValue(slaveName=f"{ts_id}__SHIP_MODEL",
                                                             slaveVar="north")
            tar_east            = self.instance.GetLastValue(slaveName=f"{ts_id}__SHIP_MODEL",
                                                             slaveVar="east")
            
            # Compute the hypotenuse
            d_north             = own_north - tar_north
            d_east              = own_east - tar_east
            dist                = np.hypot(d_north, d_east)
            
            # Add to the distance list
            dist_dict[ts_id]    = dist
        
        return dist_dict
    
    
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
        # Initial request flag
        any_request = False
        
        # Start recording
        self.nearest_dist_dict  = self._get_distances_to_own_ship()

        while not any_request:
            # Simulator integration
            self._step(action_dict)
            
            # Set action dict as None once used to enable 
            # simulator step up without action
            if isinstance(action_dict, dict):
                action_dict = None
                
            # Record target ships' nearest distance to own ship
            nearest_dist_dict_temp = self._get_distances_to_own_ship()
            self.nearest_dist_dict = {
                ts_id: min(old_val, nearest_dist_dict_temp[ts_id])
                for ts_id, old_val in self.nearest_dist_dict.items()
            }

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
        
        # Get the denormalized version of action
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
        terminated              = self.terminated
        truncated               = self.truncated
        info                    = self._get_info(
                                    action_masks=action_masks,
                                    ts_iw_id_masked=ts_iw_id_masked,
                                    action_masked=action_masked
                                    )
        
        # Reward computation
        scope_angles            = [act[0] for act in action]
        prev_scope_angles       = self.prev_scope_angels
        IW_sampling_data        = [self.instance.IW_sampling_data[sid] for sid in self.ts_iw_id]
        last_frame              = [next(reversed(datum)) for datum in IW_sampling_data]
        IW_coordinates          = [datum[frame]["sampled_inter_wps"][-1] for datum, frame in zip(IW_sampling_data, last_frame)]
        args                    = (self.os_id, self.ts_id, self.ts_iw_idx,
                                   self.instance.stop_info, self.n_ts,
                                   self.reward_components, self.finish_intercept_flags,
                                   self.skip_map_evaluation,
                                   self.ts_iw_id , self.nearest_dist_dict,
                                   self.remaining_requests_bound,
                                   self.max_scope_angles, IW_coordinates,
                                   scope_angles, prev_scope_angles,
                                   action_masks, self.routes_cog_ned_deg)
        reward                  = compute_reward(self._denormalize_observation(observation), args)     # Use denormalized observation
        
        # Assign the currently sampled scope angle to the previous scope angle holder variable
        self.prev_scope_angels  = scope_angles
        
        # Record state-action transition
        self.obs_list.append(observation)
        self.action_list.append(action_norm)
        self.reward_list.append(reward)
        self.terminated_list.append(terminated)
        self.truncated_list.append(truncated)
        self.info_list.append(truncated)
        self.event_timestamp_list.append(self.instance.time)

        return observation, reward, terminated, truncated, info
    
    
    def reset(self, seed: Optional[int] = None, options: Optional[dict] = None, specific_case_idx=None):
        """
            Reset the environment all together
        """
        # First empty the instance if it exists already
        if getattr(self, "instance", None) is not None:
            self.instance = None
        
        # IMPORTANT: Seed the random number generator
        super().reset(seed=seed)
        self.np_random, _ = gym.utils.seeding.np_random(seed)
        
        # Do repeated attempts of reset until an event happened
        max_attempts = 20

        for attempt in range(max_attempts):            
            # Run the simulator until at least one of the enabled ship request captaint intent
            self.terminated         = False
            self.truncated          = False
            
            # Sample the case index depends on the training/evaluation mode
            if specific_case_idx is not None:
                case_idx                = specific_case_idx
            elif self.for_training:
                case_idx                = int(self.np_random.integers(0, self.start_eval_case_id))
            else:
                case_idx                = int(self.np_random.integers(self.start_eval_case_id, self.n_spawn_cases))
            
            # Use Ship Traffic Generator to generates collision encounter case
            config                      = load_base_config(self.config_path)
            case                        = self.spawn_cases[case_idx]
            spawn_requests              = case["spawn_requests"]
            own_ship_initial            = case["own_ship_initial"]
            encounters                  = case["encounters"]
            
            # For output
            self.case_idx               = case_idx
            self.spawn_requests         = spawn_requests
            self.own_ship_initial       = own_ship_initial
            self.encounters             = encounters 
            
            # Spawn request
            self.routes_cog_ned_deg     = []
            
            # Initial intercept scope angle pass flag
            self.finish_intercept_flags = [False] * self.n_ts_iw
            
            for ts_id in self.ts_iw_id:
                north_route         = spawn_requests[ts_id]["north_route"]
                east_route          = spawn_requests[ts_id]["east_route"]
                
                d_north             = north_route[1] - north_route[0]
                d_east              = east_route[1]  - east_route[0]
                
                route_cog_ned_deg   = np.rad2deg(np.atan2(d_east, d_north))
                
                self.routes_cog_ned_deg.append(route_cog_ned_deg)
                

            # Instantiate the ShipInTransitCoSimulation
            # FMPy
            if self.use_fmpy:
                if self.instance is None:
                    self.instance = self.orchestrator(
                        config=config,
                        spawn_requests=spawn_requests,
                        ROOT=self.ROOT,
                        skip_map_evaluation=self.skip_map_evaluation,
                        IW_sampling_animated=True
                    )
                else:
                    self.instance.RespawnAndReset(config=config,
                                                  spawn_requests=spawn_requests,
                                                  ROOT=self.ROOT)
            # Libcosimpy
            else:
                self.instance = self.orchestrator(
                    config=config,
                    spawn_requests=spawn_requests,
                    ROOT=self.ROOT,
                    skip_map_evaluation=self.skip_map_evaluation,
                    IW_sampling_animated=True
                )
            
            # List for recording one episode
            self.obs_list                   = []
            self.action_list                = []
            self.terminated_list            = []
            self.truncated_list             = []
            self.info_list                  = []
            self.event_timestamp_list       = []
            
            # Reward container
            self.reward_list                            = []
            self.reward_components = {
                "own_ship_collision_rewards"                : [],
                "own_ship_grounding_rewards"                : [],
                "own_ship_navigational_failure_rewards"     : [],
                "own_ship_reaches_end_waypoint_rewards"     : [],
                "tar_ships_collision_rewards"               : [],
                "tar_ships_grounding_rewards"               : [],
                "tar_ships_navigation_failure_rewards"      : [],
                "tar_ships_reaches_end_waypoint_rewards"    : [],
                "nearest_distance_rewards"                  : [],
                "nearest_distance_iw_rewards"               : [],
                "nearest_distance_roa_rewards"              : [],
                "total_distance_rewards"                    : [],
                "scope_angle_request_done_rewards"          : [],
                "scope_angle_change_log_likelihood_rewards" : [],  
                "intercept_scope_angle_rewards"             : [],
            }
            
            # Prev_action recorder (initiated at 0.0 degree)
            self.prev_scope_angels          = [0.0] * self.n_ts_iw
            
            # Advance the simulation until an event is found
            found_event = self._advance_until_next_event()

            # If an event is encountered, output observation and info as a valid
            # reset attempt
            if found_event:
                observation = self._get_obs()
                info = self._get_info(reset_attempts=attempt + 1,
                                      case_idx=case_idx)
                
                # Record
                self.obs_list.append(observation)
                self.info_list.append(info)
                self.event_timestamp_list.append(self.instance.time)
                return observation, info

        raise RuntimeError(
            "Failed to initialize an episode with a captain-intent request "
            f"after {max_attempts} attempts."
        )