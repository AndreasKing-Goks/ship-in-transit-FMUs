"""
Simple Collision Avoidance Python FMU implementation.
This FMU manages the simple collision avoidance algorithm.

Authors : Andreas R.G. Sitorus
Date    : February 2026
"""

from pythonfmu import Fmi2Causality, Fmi2Slave, Fmi2Variability, Real, Integer, Boolean, String
import numpy as np
import traceback

class SimpleCollisionAvoidance(Fmi2Slave):
    
    author = "Andreas R.G. Sitorus"
    description = "Simple Collision Avoidance Algorithm Python FMU Implementation"
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
         
        ## Parameters
        self.throttle_scale_factor  = 0.5
        self.rud_ang_increment_deg  = 15.0
        self.danger_zone_radius     = 926.0     # 0.5 nautical mile
        self.collision_zone_radius  = 100.0     # Ideally ship length
        self.max_target_ship_count  = 0         # Up to three
        self.hold_time              = 600.0     # Max time for holding one target ship
        
        ## Input
        self.own_north              = 0.0
        self.own_east               = 0.0
        self.own_yaw_angle          = 0.0
        self.own_measured_speed     = 0.0
        
        self.tar_1_north            = 0.0
        self.tar_1_east             = 0.0
        self.tar_1_yaw_angle        = 0.0
        self.tar_1_measured_speed   = 0.0
        
        self.tar_2_north            = 0.0
        self.tar_2_east             = 0.0
        self.tar_2_yaw_angle        = 0.0
        self.tar_2_measured_speed   = 0.0
        
        self.tar_3_north            = 0.0
        self.tar_3_east             = 0.0
        self.tar_3_yaw_angle        = 0.0
        self.tar_3_measured_speed   = 0.0
        
        self.throttle_cmd           = 0.0
        self.rudder_angle_deg       = 0.0
        
        ## Output
        self.new_throttle_cmd       = 0.0 
        self.new_rudder_angle_deg   = 0.0
        self.colav_active           = False
        self.ship_collision         = False
        
        # Internal Variable
        self.prioritize_one_target  = False
        self.held_target_idx        = 0
        self.timer                  = 0.0
        
        ## Registration
        # Parameters
        self.register_variable(Real("throttle_scale_factor", causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))
        self.register_variable(Real("rud_ang_increment_deg", causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))
        self.register_variable(Real("danger_zone_radius", causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))
        self.register_variable(Real("collision_zone_radius", causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))
        self.register_variable(Integer("max_target_ship_count", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("hold_time", causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))
        
        # Input
        self.register_variable(Real("throttle_cmd", causality=Fmi2Causality.input))
        self.register_variable(Real("rudder_angle_deg", causality=Fmi2Causality.input))
        
        self.register_variable(Real("own_north", causality=Fmi2Causality.input))
        self.register_variable(Real("own_east", causality=Fmi2Causality.input))
        self.register_variable(Real("own_yaw_angle", causality=Fmi2Causality.input))
        self.register_variable(Real("own_measured_speed", causality=Fmi2Causality.input))
        
        self.register_variable(Real("tar_1_north", causality=Fmi2Causality.input))
        self.register_variable(Real("tar_1_east", causality=Fmi2Causality.input))
        self.register_variable(Real("tar_1_yaw_angle", causality=Fmi2Causality.input))
        self.register_variable(Real("tar_1_measured_speed", causality=Fmi2Causality.input))
        
        self.register_variable(Real("tar_2_north", causality=Fmi2Causality.input))
        self.register_variable(Real("tar_2_east", causality=Fmi2Causality.input))
        self.register_variable(Real("tar_2_yaw_angle", causality=Fmi2Causality.input))
        self.register_variable(Real("tar_2_measured_speed", causality=Fmi2Causality.input))
        
        self.register_variable(Real("tar_3_north", causality=Fmi2Causality.input))
        self.register_variable(Real("tar_3_east", causality=Fmi2Causality.input))
        self.register_variable(Real("tar_3_yaw_angle", causality=Fmi2Causality.input))
        self.register_variable(Real("tar_3_measured_speed", causality=Fmi2Causality.input))
        
        # Output
        self.register_variable(Real("new_throttle_cmd", causality=Fmi2Causality.output))
        self.register_variable(Real("new_rudder_angle_deg", causality=Fmi2Causality.output))
        self.register_variable(Boolean("colav_active", causality=Fmi2Causality.output))
        self.register_variable(Boolean("ship_collision", causality=Fmi2Causality.output))
        
    
    def _wrap_to_pi(self, a):
        return (a + np.pi) % (2*np.pi) - np.pi
    
    
    def do_step(self, current_time: float, step_size: float) -> bool:
        try:
            ### INITIATE
            self.p_own                  = np.array([self.own_north, self.own_east], dtype=float)
            self.v_own                  = self.own_measured_speed * np.array([np.cos(self.own_yaw_angle), np.sin(self.own_yaw_angle)])
            self.p_tar_list             = []
            self.psi_tar_list           = []
            self.v_tar_list             = []
            self.beta_list              = []
            self.tcpa_list              = []
            self.dcpa_list              = []
            self.range_rate_list        = []
            
            c = int(self.max_target_ship_count)
            c = max(0, min(c,3))
            
            for i in range (1, c+1):
                north = getattr(self, f"tar_{i}_north")
                east  = getattr(self, f"tar_{i}_east")
                yaw   = getattr(self, f"tar_{i}_yaw_angle")
                speed = getattr(self, f"tar_{i}_measured_speed")

                p_tar = np.array([north, east], dtype=float)
                v_tar = speed * np.array([np.cos(yaw), np.sin(yaw)], dtype=float)
                
                self.p_tar_list.append(p_tar)
                self.psi_tar_list.append(yaw)
                self.v_tar_list.append(v_tar)
                
            ### Compute Relative Bearing (Beta), TCPA, and DCPA, Range Rate
            for i in range(c):
                # Relative distance
                r = self.p_tar_list[i] - self.p_own
                
                # RELATIVE VELOCITY
                v = self.v_tar_list[i] - self.v_own
                
                # Relative Bearing
                theta = np.arctan2(r[1], r[0])
                beta  = self._wrap_to_pi(theta - self.own_yaw_angle)
                self.beta_list.append(beta)
                
                ## TCPA and DCPA
                v_norm_sq = np.dot(v, v)
                
                # If the squared norm of the speed is close to 0
                if v_norm_sq < 1e-6:
                    tcpa = None                # Both ships are stationary to each other
                    dcpa = np.linalg.norm(r)
                else:
                    tcpa = - np.dot(r,v) / v_norm_sq
                    dcpa = np.linalg.norm(r + v * tcpa)
                
                self.tcpa_list.append(tcpa)
                self.dcpa_list.append(dcpa)
                
                ## RANGE RATE
                dist = np.linalg.norm(r)
                if dist < 1e-6:
                    range_rate = 0.0
                else:
                    r_hat = r / dist
                    range_rate = float(np.dot(r_hat, v))
                self.range_rate_list.append(range_rate)
                
            ### COLAV DECISION
            dcpa_arr    = np.array(self.dcpa_list, dtype=float)
            rr_arr      = np.array(self.range_rate_list, dtype=float)
            
            tcpa_check          = np.array([(t is not None) and (t > 0.0) for t in self.tcpa_list], dtype=bool)
            dcpa_check          = dcpa_arr < self.danger_zone_radius
            rr_check            = rr_arr < 0.0
            
            # Colav status
            # Active if there exists target with:
            # tcpa > 0                  -> Collision will happen in the future
            # dcpa < danger_zone_radius -> Violate the safety region
            # rr   < 0 (ideally)        -> Ideally is closing in
            
            # Set the colav flag
            colav_active_status = tcpa_check & dcpa_check & rr_check    # OPERATOR "&" IS FOR NUMPY ARRAYS
            self.colav_active   = bool(np.any(colav_active_status))     # OUTPUT
            
            # Set the collision status
            collision_status    = dcpa_arr < self.collision_zone_radius
            self.ship_collision = bool(np.any(collision_status))    # OUTPUT
            
            if self.ship_collision:
                self.new_throttle_cmd       = float(np.clip(self.throttle_cmd, 0.0, 1.0))   # OUTPUT, clamp after scaling
                self.new_rudder_angle_deg   = self.rudder_angle_deg         # OUTPUT
                
                return True
            
            # Score the threat level for each target ship
            candidates = []
            for i in range(c):
                tcpa = self.tcpa_list[i]
                dcpa = self.dcpa_list[i]
                rr   = self.range_rate_list[i]
                
                if tcpa is None or tcpa <= 0:
                    continue
                if rr >= 0:
                    continue
                if dcpa > self.danger_zone_radius:
                    continue
                
                # Risk score
                score = (1.0/(tcpa+1e-3) + (1.0/(dcpa+1e-3)) + 0.1*(-rr))
                candidates.append((score,i))
            
            # Index Hold Logic
            def is_still_candidate(i):
                tcpa = self.tcpa_list[i]
                return (tcpa is not None) and (tcpa > 0.0) and (self.range_rate_list[i] < 0.0) and (self.dcpa_list[i] <= self.danger_zone_radius)

            # If the prioritized target is no longer a threat, (or the held target index is invalid) release the hold
            if self.prioritize_one_target:
                if (self.held_target_idx < 0) or (self.held_target_idx >= c) or (not is_still_candidate(self.held_target_idx)):
                    self.prioritize_one_target = False
                    self.timer = 0.0
            
            # Get the maximum score, then respons, else do nothing
            if candidates:
                # _, idx = max(candidates)
                idx = max(candidates, key=lambda x: x[0])[1]
                
                ## Prioritize one_target if multiple ship in vicinity (and not yet prioritizing one target)
                # Start tracking the target
                if len(candidates) > 1 and (not self.prioritize_one_target):
                    self.held_target_idx = idx
                    self.prioritize_one_target = True   # PRIORITIZING ONLY HAPPEN WHEN WE HAVE >1 CANDIDATES
                    self.timer = 0.0
                
                # If prioritize one target use held_target_idx, else use idx
                use_idx = self.held_target_idx if self.prioritize_one_target else idx
                
                beta = self.beta_list[use_idx]
                colav_rudder_sign = float(-np.sign(beta)) if abs(beta) > 1e-6 else -1.0
                
                # OUTPUTS
                self.new_throttle_cmd       = float(np.clip(self.throttle_scale_factor * self.throttle_cmd, 0.0, 1.0))  # clamp after scaling
                self.new_rudder_angle_deg   = colav_rudder_sign * self.rud_ang_increment_deg + self.rudder_angle_deg
                    
                if self.prioritize_one_target:                        
                    self.timer += step_size
                    if self.timer >= self.hold_time:
                        self.prioritize_one_target = False
                        self.timer = 0.0
            
            else:
                # OUTPUTS
                self.new_throttle_cmd       = float(np.clip(self.throttle_cmd, 0.0, 1.0))   # OUTPUT, clamp after scaling
                self.new_rudder_angle_deg   = self.rudder_angle_deg     # OUTPUT
            
        except Exception as e:
            # IMPORTANT: do not crash host
            print(f"[SimpleCollisionAvoidance] ERROR t={current_time} dt={step_size}: {type(e).__name__}: {e}")
            print(traceback.format_exc())
            
            # Freeze dynamics (keep last state/outputs)
            self.new_throttle_cmd       = float(np.clip(self.throttle_cmd, 0.0, 1.0))     # clamp after scaling
            self.new_rudder_angle_deg   = self.rudder_angle_deg
            self.colav_active           = False
            self.ship_collision         = False
            
        return True
            