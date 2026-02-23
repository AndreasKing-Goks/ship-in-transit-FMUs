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
        self.T_lookahead            = 900.0     # Soon to collide  (second)
        
        ## Input
        self.own_north              = 0.0
        self.own_east               = 0.0
        self.own_yaw_angle          = 0.0
        self.own_measured_speed     = 0.0
        
        for i in range(1,4):
            setattr(self, f"tar_{i}_north", 0.0)
            setattr(self, f"tar_{i}_east", 0.0)
            setattr(self, f"tar_{i}_yaw_angle", 0.0)
            setattr(self, f"tar_{i}_measured_speed", 0.0)
        
        self.throttle_cmd           = 0.0
        self.rudder_angle_deg       = 0.0
        
        ## Output
        self.new_throttle_cmd        = 0.0 
        self.new_rudder_angle_deg    = 0.0
        self.colav_rud_ang_increment = 0.0 # OUTPUT
        self.colav_active            = False
        self.ship_collision          = False
        
        for i in range (1,4):
            setattr(self, f"beta_own_to_tar_{i}", 0.0)
            setattr(self, f"tcpa_own_to_tar_{i}", 0.0)
            setattr(self, f"dcpa_own_to_tar_{i}", 0.0)
            setattr(self, f"dist_own_to_tar_{i}", 0.0)
            setattr(self, f"rr_own_to_tar_{i}", 0.0)
        
        # Internal Variable
        self.prioritize_one_target  = False
        self.held_target_idx        = 0
        self.timer                  = 0.0
        self.colav_rudder_sign      = -1.0
        
        ## Registration
        # Parameters
        self.register_variable(Real("throttle_scale_factor", causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))
        self.register_variable(Real("rud_ang_increment_deg", causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))
        self.register_variable(Real("danger_zone_radius", causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))
        self.register_variable(Real("collision_zone_radius", causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))
        self.register_variable(Integer("max_target_ship_count", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("hold_time", causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))
        self.register_variable(Real("T_lookahead", causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))
        
        # Input
        self.register_variable(Real("throttle_cmd", causality=Fmi2Causality.input))
        self.register_variable(Real("rudder_angle_deg", causality=Fmi2Causality.input))
        
        self.register_variable(Real("own_north", causality=Fmi2Causality.input))
        self.register_variable(Real("own_east", causality=Fmi2Causality.input))
        self.register_variable(Real("own_yaw_angle", causality=Fmi2Causality.input))
        self.register_variable(Real("own_measured_speed", causality=Fmi2Causality.input))
        
        for i in range(1,4):
            self.register_variable(Real(f"tar_{i}_north", causality=Fmi2Causality.input))
            self.register_variable(Real(f"tar_{i}_east", causality=Fmi2Causality.input))
            self.register_variable(Real(f"tar_{i}_yaw_angle", causality=Fmi2Causality.input))
            self.register_variable(Real(f"tar_{i}_measured_speed", causality=Fmi2Causality.input))    
        
        # Output
        self.register_variable(Real("new_throttle_cmd", causality=Fmi2Causality.output))
        self.register_variable(Real("new_rudder_angle_deg", causality=Fmi2Causality.output))
        self.register_variable(Real("colav_rud_ang_increment", causality=Fmi2Causality.output))
        self.register_variable(Boolean("colav_active", causality=Fmi2Causality.output))
        self.register_variable(Boolean("ship_collision", causality=Fmi2Causality.output))
        
        for i in range(1,4):
            self.register_variable(Real(f"beta_own_to_tar_{i}", causality=Fmi2Causality.output))
            self.register_variable(Real(f"tcpa_own_to_tar_{i}", causality=Fmi2Causality.output))
            self.register_variable(Real(f"dcpa_own_to_tar_{i}", causality=Fmi2Causality.output))
            self.register_variable(Real(f"dist_own_to_tar_{i}", causality=Fmi2Causality.output))
            self.register_variable(Real(f"rr_own_to_tar_{i}", causality=Fmi2Causality.output))
        
    
    def _wrap_to_pi(self, a):
        return (a + np.pi) % (2*np.pi) - np.pi
    
    
    def do_step(self, current_time: float, step_size: float) -> bool:
        try:
            # To take into account issue with all-zero initial value, 
            # Turn off the COLAV during the very first time step
            if current_time < step_size:
                self.new_throttle_cmd       = 0.0 
                self.new_rudder_angle_deg   = 0.0
                self.colav_active           = False
                self.ship_collision         = False
                
                for i in range (1,4):
                    setattr(self, f"beta_own_to_tar_{i}", 0.0)
                    setattr(self, f"tcpa_own_to_tar_{i}", 0.0)
                    setattr(self, f"dcpa_own_to_tar_{i}", 0.0)
                    setattr(self, f"dist_own_to_tar_{i}", 0.0)
                    setattr(self, f"rr_own_to_tar_{i}", 0.0)
                
                return True
            
            ### INITIATE
            p_own                  = np.array([self.own_north, self.own_east], dtype=float)
            v_own                  = self.own_measured_speed * np.array([np.cos(self.own_yaw_angle), np.sin(self.own_yaw_angle)])
            p_tar_list             = []
            psi_tar_list           = []
            v_tar_list             = []
            beta_list              = []
            tcpa_list              = []
            dcpa_list              = []
            dist_list              = []
            range_rate_list        = []
            
            c = int(self.max_target_ship_count)
            c = max(0, min(c,3))
            
            for i in range (1, c+1):
                north = getattr(self, f"tar_{i}_north")
                east  = getattr(self, f"tar_{i}_east")
                yaw   = getattr(self, f"tar_{i}_yaw_angle")
                speed = getattr(self, f"tar_{i}_measured_speed")

                p_tar = np.array([north, east], dtype=float)
                v_tar = speed * np.array([np.cos(yaw), np.sin(yaw)], dtype=float)
                
                p_tar_list.append(p_tar)
                psi_tar_list.append(yaw)
                v_tar_list.append(v_tar)
                
            ### Compute Relative Bearing (Beta), TCPA, and DCPA, Range Rate
            for i in range(c):
                # Relative distance
                r = p_tar_list[i] - p_own
                
                # RELATIVE VELOCITY
                v = v_tar_list[i] - v_own
                
                # Relative Bearing
                theta = np.arctan2(r[1], r[0])
                beta  = self._wrap_to_pi(theta - self.own_yaw_angle)
                setattr(self, f"beta_own_to_tar_{i+1}", beta)
                beta_list.append(beta)
                
                ## TCPA and DCPA
                v_norm_sq = np.dot(v, v)
                
                # If the squared norm of the speed is close to 0
                if v_norm_sq < 1e-6:
                    tcpa = np.nan               # Both ships are stationary to each other, signify as nan
                    dcpa = np.linalg.norm(r)
                else:
                    tcpa = - np.dot(r,v) / v_norm_sq
                    dcpa = np.linalg.norm(r + v * tcpa)
                    
                setattr(self, f"tcpa_own_to_tar_{i+1}", tcpa)
                setattr(self, f"dcpa_own_to_tar_{i+1}", dcpa)
                
                tcpa_list.append(tcpa)
                dcpa_list.append(dcpa)
                
                ## RANGE RATE
                dist = np.linalg.norm(r)
                dist_list.append(dist)
                setattr(self, f"dist_own_to_tar_{i+1}", dist)
                if dist < 1e-6:
                    range_rate = 0.0
                else:
                    r_hat = r / dist
                    range_rate = float(np.dot(r_hat, v))
                setattr(self, f"rr_own_to_tar_{i+1}", range_rate)
                range_rate_list.append(range_rate)
                
            ### COLAV DECISION
            # Set the collision status
            dist_arr    = np.array(dist_list, dtype=float)
            collision_status    = dist_arr <= self.collision_zone_radius
            self.ship_collision = bool(np.any(collision_status))    # OUTPUT
            
            if self.ship_collision:
                self.new_throttle_cmd       = float(np.clip(self.throttle_cmd, 0.0, 1.0))   # OUTPUT, clamp after scaling
                self.new_rudder_angle_deg   = self.rudder_angle_deg         # OUTPUT
                
                return True
            
            # Score the threat level for each target ship
            candidates = []
            for i in range(c):
                tcpa = tcpa_list[i]
                dcpa = dcpa_list[i]
                rr   = range_rate_list[i]
                
                # Colav status
                # Active if there exists target with (in order):
                # dcpa < danger_zone_radius -> If it going to violate the safety region
                # tcpa > 0                  -> If collision will happen in the future
                # tcpa < T_lookahead        -> If collision will happen soon
                # rr   < 0 (ideally)        -> Ideally is closing in
                
                # Initial colav status
                self.colav_active   = False     # OUTPUT
                
                if dcpa > self.danger_zone_radius:
                    continue
                if tcpa is None or tcpa <= 0:
                    continue
                if tcpa > self.T_lookahead:
                    continue
                if rr >= 0:
                    continue
                
                # Set colav is active if it passed through the logic gate
                self.colav_active   = True     # OUTPUT
                
                # Risk score
                score = (1.0/(tcpa+1e-3) + (1.0/(dcpa+1e-3)) + 0.1*(-rr))
                candidates.append((score,i))
            
            # Index Hold Logic
            def is_still_candidate(i):
                tcpa = tcpa_list[i]
                return (tcpa is not None) and (tcpa > 0.0) and (range_rate_list[i] < 0.0) and (dcpa_list[i] <= self.danger_zone_radius)

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
                    
                    # Also held the colav rudder sign
                    beta = beta_list[use_idx]
                    self.colav_rudder_sign      = float(-np.sign(beta)) if abs(beta) > 1e-6 else -1.0
                
                # If prioritize one target use held_target_idx, else use idx
                use_idx = self.held_target_idx if self.prioritize_one_target else idx
                
                # Accrue the rudder angle increment
                self.colav_rud_ang_increment += self.colav_rudder_sign * self.rud_ang_increment_deg # OUTPUT
                
                # OUTPUTS
                self.new_throttle_cmd        = float(np.clip(self.throttle_scale_factor * self.throttle_cmd, 0.0, 1.0))  # clamp after scaling
                self.new_rudder_angle_deg    =  self.rudder_angle_deg + self.colav_rud_ang_increment
                    
                if self.prioritize_one_target:                        
                    self.timer += step_size
                    if self.timer >= self.hold_time:
                        self.prioritize_one_target = False
                        self.timer = 0.0
            
            else:
                # OUTPUTS
                self.new_throttle_cmd        = float(np.clip(self.throttle_cmd, 0.0, 1.0))   # OUTPUT, clamp after scaling
                self.new_rudder_angle_deg    = self.rudder_angle_deg     # OUTPUT
                self.colav_rud_ang_increment = 0.0 # OUTPUT
            
        except Exception as e:
            # IMPORTANT: do not crash host
            print(f"[SimpleCollisionAvoidance] ERROR t={current_time} dt={step_size}: {type(e).__name__}: {e}")
            print(traceback.format_exc())
            
            # Freeze dynamics (keep last state/outputs)
            self.new_throttle_cmd        = float(np.clip(self.throttle_cmd, 0.0, 1.0))     # clamp after scaling
            self.new_rudder_angle_deg    = self.rudder_angle_deg
            self.colav_rud_ang_increment = 0.0 # OUTPUT
            self.colav_active            = False
            self.ship_collision          = False
            
        return True
            