"""
COLREGS Compliant Simple Collision Avoidance Python FMU implementation.
This FMU manages the simple collision avoidance algorithm.

Authors : Andreas R.G. Sitorus
Date    : October 2026
"""

from pythonfmu import Fmi2Causality, Fmi2Slave, Fmi2Variability, Real, Integer, Boolean, String
import numpy as np
import traceback

class CollisionAvoidanceCOLREGS(Fmi2Slave):
    
    author = "Andreas R.G. Sitorus"
    description = "COLREGS-compliant Collision Avoidance Algorithm Python FMU Implementation"
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
         
        ## Parameters
        self.soft_heading_offset_deg        = 30            # Soft heading offset deg
        self.hard_heading_offset_deg        = 45            # Hard heading offset deg
        self.danger_zone_radius             = 926.0         # 0.5 nautical mile
        self.collision_zone_radius          = 100.0         # Ideally ship length
        self.max_target_ship_count          = 0             # Up to three
        self.hold_time                      = 600.0         # Max time for holding one target ship
        self.T_lookahead                    = 900.0         # Soon to collide  (second)
        self.own_ship_sog_max               = 6.0           # in m/s
        self.turning_direction              = "STARBOARD"
        self.colav_tag                      = "colregs-compliant_colav"
        
        ## Input
        self.own_north                      = 0.0
        self.own_east                       = 0.0
        self.own_yaw_angle                  = 0.0
        self.own_speed_north                = 0.0
        self.own_speed_east                 = 0.0
        
        for i in range(1,4):
            # For each ship, set the initial position to be far away, facing north, and static
            setattr(self, f"tar_{i}_north", 1e9)
            setattr(self, f"tar_{i}_east", 1e9)
            setattr(self, f"tar_{i}_speed_north", 0.0)
            setattr(self, f"tar_{i}_speed_east", 0.0)
        
        ## Output
        self.colav_speed_reduction_factor   = 1.0 
        self.colav_heading_offset_deg       = 0.0 # OUTPUT
        self.colav_active                   = False
        self.ship_collision                 = False
        self.ship_priority_idx              = -1
        self.encounter_status               = "None"
        self.role_status                    = "None"
        
        for i in range (1,4):
            setattr(self, f"beta_own_to_tar_{i}", 0.0)
            setattr(self, f"beta_tar_to_own_{i}", 0.0)
            setattr(self, f"tcpa_own_to_tar_{i}", 0.0)
            setattr(self, f"dcpa_own_to_tar_{i}", 0.0)
            setattr(self, f"dist_own_to_tar_{i}", 0.0)
            setattr(self, f"rr_own_to_tar_{i}", 0.0)
        
        # Internal Variable
        self.prioritize_one_target          = False
        self.held_target_idx                = 0
        self.timer                          = 0.0
        self.STARBOARD                      = +1.0
        self.PORT                           = -1.0
        
        ## Registration
        # Parameters
        self.register_variable(Real("soft_heading_offset_deg", causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))
        self.register_variable(Real("hard_heading_offset_deg", causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))
        self.register_variable(Real("danger_zone_radius", causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))
        self.register_variable(Real("collision_zone_radius", causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))
        self.register_variable(Integer("max_target_ship_count", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("hold_time", causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))
        self.register_variable(Real("T_lookahead", causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))
        self.register_variable(Real("own_ship_sog_max", causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))
        self.register_variable(String("turning_direction", causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))
        self.register_variable(String("colav_tag", causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))
        
        # Input        
        self.register_variable(Real("own_north", causality=Fmi2Causality.input))
        self.register_variable(Real("own_east", causality=Fmi2Causality.input))
        self.register_variable(Real("own_yaw_angle", causality=Fmi2Causality.input))
        self.register_variable(Real("own_speed_north", causality=Fmi2Causality.input))
        self.register_variable(Real("own_speed_east", causality=Fmi2Causality.input))
        
        for i in range(1,4):
            self.register_variable(Real(f"tar_{i}_north", causality=Fmi2Causality.input))
            self.register_variable(Real(f"tar_{i}_east", causality=Fmi2Causality.input))
            self.register_variable(Real(f"tar_{i}_speed_north", causality=Fmi2Causality.input))
            self.register_variable(Real(f"tar_{i}_speed_east", causality=Fmi2Causality.input))    
        
        # Output
        self.register_variable(Real("colav_speed_reduction_factor", causality=Fmi2Causality.output))
        self.register_variable(Real("colav_heading_offset_deg", causality=Fmi2Causality.output))
        self.register_variable(Integer("ship_priority_idx", causality=Fmi2Causality.output, variability=Fmi2Variability.discrete))
        self.register_variable(Boolean("colav_active", causality=Fmi2Causality.output, variability=Fmi2Variability.discrete))
        self.register_variable(Boolean("ship_collision", causality=Fmi2Causality.output, variability=Fmi2Variability.discrete))
        self.register_variable(String("encounter_status", causality=Fmi2Causality.output, variability=Fmi2Variability.discrete))
        self.register_variable(String("role_status", causality=Fmi2Causality.output, variability=Fmi2Variability.discrete))
        
        for i in range(1,4):
            self.register_variable(Real(f"beta_own_to_tar_{i}", causality=Fmi2Causality.output))
            self.register_variable(Real(f"beta_tar_to_own_{i}", causality=Fmi2Causality.output))
            self.register_variable(Real(f"tcpa_own_to_tar_{i}", causality=Fmi2Causality.output))
            self.register_variable(Real(f"dcpa_own_to_tar_{i}", causality=Fmi2Causality.output))
            self.register_variable(Real(f"dist_own_to_tar_{i}", causality=Fmi2Causality.output))
            self.register_variable(Real(f"rr_own_to_tar_{i}", causality=Fmi2Causality.output))
        
    
    def _wrap_to_pi(self, a):
        return (a + np.pi) % (2*np.pi) - np.pi
    
    
    def COLREG_situation_assessment(self, beta_own_to_tar, beta_tar_to_own, range_rate, risk_score, turning_direction:String="STARBOARD"):
        # Conditions
        beta_own_to_tar_deg         = np.rad2deg(beta_own_to_tar)
        beta_tar_to_own_deg         = np.rad2deg(beta_tar_to_own)
        closing                     = range_rate < 0.0
        own_is_overtaking_tar       = (np.isfinite(beta_tar_to_own_deg)
                                       and np.abs(beta_tar_to_own_deg) > 112.5 
                                       and closing)                                                             # Own ship is behind and closing in
        tar_is_overtaking_own       = (np.isfinite(beta_own_to_tar_deg)
                                       and np.abs(beta_own_to_tar_deg) > 112.5 
                                       and closing)                                                             # Target ship is behind and closing in
        is_head_on                  = np.abs(beta_own_to_tar_deg) < 5.0 and np.abs(beta_tar_to_own_deg) < 5.0   # Frontal encounter
        tar_on_starboard            = beta_own_to_tar_deg >= 5.0 and beta_own_to_tar_deg <= 112.5               # Target on starboard. O deg points North, CW
        tar_on_port                 = beta_own_to_tar_deg <= -5.0 and beta_own_to_tar_deg >= -112.5             # Target on starboard. O deg points North, CW 
        
        encounter                   = "None"
        role                        = "None"
        heading_offset_deg          = 0.0
        speed_reduction_factor      = 1.0
        
        if turning_direction == "PORT":
            direction       = self.PORT
            opst_direction  = self.STARBOARD
        else:
            direction       = self.STARBOARD
            opst_direction  = self.PORT
        
        # Overtaking give-way
        if own_is_overtaking_tar:
            encounter               = "OVERTAKING"
            role                    = "GIVE_WAY"
            # Near collision -> STOP, turn hard
            if risk_score > 0.95:
                heading_offset_deg      = direction * self.hard_heading_offset_deg
                speed_reduction_factor  = 0.0
            # High danger -> Half speed, hard heading offset
            if 0.85 <risk_score <= 0.95:
                heading_offset_deg      = direction * self.hard_heading_offset_deg
                speed_reduction_factor  = 0.5
            # Moderate danger -> Full speed, hard heading offset
            elif 0.75 < risk_score <= 0.85:
                heading_offset_deg      = direction * self.hard_heading_offset_deg
                speed_reduction_factor  = 1.0
            # Low danger -> Full speed, soft heading offset
            elif 0.6 < risk_score <= 0.75:
                heading_offset_deg      = direction * self.soft_heading_offset_deg
                speed_reduction_factor  = 1.0
            # Warning -> Full speed, no heading offset
            elif risk_score <= 0.6:
                heading_offset_deg      = 0.0
                speed_reduction_factor  = 1.0
        
        # Overtaking stand-on
        elif tar_is_overtaking_own:
            encounter               = "OVERTAKING"
            role                    = "STAND_ON"
            heading_offset_deg      = 0.0
            speed_reduction_factor  = 1.0
            # Warning -> Half speed, hard heading offset
            if risk_score > 0.6:
                heading_offset_deg      = direction * self.hard_heading_offset_deg
                speed_reduction_factor  = 0.5      
            
        # Head on
        elif is_head_on:
            encounter               = "HEAD_ON"
            role                    = "GIVE_WAY"
            # Near collision -> STOP, turn hard
            if risk_score > 0.95:
                heading_offset_deg      = direction * self.hard_heading_offset_deg
                speed_reduction_factor  = 0.0
            # High danger -> Half speed, hard heading offset
            if 0.85 <risk_score <= 0.95:
                heading_offset_deg      = direction * self.hard_heading_offset_deg
                speed_reduction_factor  = 0.5
            # Moderate danger -> Full speed, hard heading offset
            elif 0.75 < risk_score <= 0.85:
                heading_offset_deg      = direction * self.hard_heading_offset_deg
                speed_reduction_factor  = 1.0
            # Low danger -> Full speed, soft heading offset
            elif 0.6 < risk_score <= 0.75:
                heading_offset_deg      = direction * self.soft_heading_offset_deg
                speed_reduction_factor  = 1.0
            # Warning -> Full speed, no heading offset
            elif risk_score <= 0.6:
                heading_offset_deg      = 0.0
                speed_reduction_factor  = 1.0
            
        # Crossing give-way
        elif tar_on_starboard:
            encounter               = "CROSSING"
            role                    = "GIVE_WAY"
            # Near collision -> STOP, turn hard
            if risk_score > 0.95:
                heading_offset_deg      = direction * self.hard_heading_offset_deg
                speed_reduction_factor  = 0.0
            # High danger -> Half speed, hard heading offset
            if 0.85 <risk_score <= 0.95:
                heading_offset_deg      = direction * self.hard_heading_offset_deg
                speed_reduction_factor  = 0.5
            # Moderate danger -> Full speed, hard heading offset
            elif 0.75 < risk_score <= 0.85:
                heading_offset_deg      = direction * self.hard_heading_offset_deg
                speed_reduction_factor  = 1.0
            # Low danger -> Full speed, soft heading offset
            elif 0.6 < risk_score <= 0.75:
                heading_offset_deg      = direction * self.soft_heading_offset_deg
                speed_reduction_factor  = 1.0
            # Warning -> Full speed, no heading offset
            elif risk_score <= 0.6:
                heading_offset_deg      = 0.0
                speed_reduction_factor  = 1.0
        
        # Crossing stand-on
        elif tar_on_port:
            encounter               = "CROSSING"
            role                    = "STAND_ON"
            heading_offset_deg      = 0.0
            speed_reduction_factor  = 1.0
            # Warning -> Half speed, hard heading offset, opposite direction
            if risk_score > 0.6:
                heading_offset_deg      = opst_direction * self.hard_heading_offset_deg
                speed_reduction_factor  = 0.5   
            
        return encounter, role, heading_offset_deg, speed_reduction_factor
    
    def do_step(self, current_time: float, step_size: float) -> bool:
        try:
            # To take into account issue with all-zero initial value, 
            # Turn off the COLAV during the very first time step
            if current_time < step_size:
                self.colav_speed_reduction_factor      = 1.0 
                self.colav_heading_offset_deg       = 0.0
                self.colav_active                   = False
                self.ship_collision                 = False
                
                for i in range (1,4):
                    setattr(self, f"beta_own_to_tar_{i}", 0.0)
                    setattr(self, f"beta_tar_to_own_{i}", 0.0)
                    setattr(self, f"tcpa_own_to_tar_{i}", 0.0)
                    setattr(self, f"dcpa_own_to_tar_{i}", 0.0)
                    setattr(self, f"dist_own_to_tar_{i}", 0.0)
                    setattr(self, f"rr_own_to_tar_{i}", 0.0)
                
                return True
            
            ### INITIATE
            p_own                   = np.array([self.own_north, self.own_east], dtype=float)
            v_own                   = np.array([self.own_speed_north, self.own_speed_east])
            p_tar_list              = []
            v_tar_list              = []
            beta_own_to_tar_list    = []
            beta_tar_to_own_list    = []
            tcpa_list               = []
            dcpa_list               = []
            dist_list               = []
            range_rate_list         = []
            
            c = int(self.max_target_ship_count)
            c = max(0, min(c,3))
            
            for i in range (1, c+1):
                north       = getattr(self, f"tar_{i}_north")
                east        = getattr(self, f"tar_{i}_east")
                speed_north = getattr(self, f"tar_{i}_speed_north")
                speed_east  = getattr(self, f"tar_{i}_speed_east")

                p_tar = np.array([north, east], dtype=float)
                v_tar = np.array([speed_north, speed_east], dtype=float)
                
                p_tar_list.append(p_tar)
                v_tar_list.append(v_tar)
                
            ### Compute Relative Bearing (Beta), TCPA, and DCPA, Range Rate
            for i in range(c):
                # Relative distance
                r = p_tar_list[i] - p_own
                
                # RELATIVE VELOCITY
                v = v_tar_list[i] - v_own
                
                # Relative Bearing Own -> Tar
                theta_own_to_tar = np.arctan2(r[1], r[0])
                beta_own_to_tar  = self._wrap_to_pi(theta_own_to_tar - self.own_yaw_angle)
                setattr(self, f"beta_own_to_tar_{i+1}", beta_own_to_tar)
                beta_own_to_tar_list.append(beta_own_to_tar)
                
                # Relative Bearing Tar -> Own
                target_speed        = np.linalg.norm(v_tar)
                
                if target_speed > 1e-3:
                    target_course       = np.arctan2(v_tar_list[i][1], v_tar_list[i][0])
                    theta_tar_to_own    = np.arctan2(-r[1], -r[0])
                    beta_tar_to_own     = self._wrap_to_pi(theta_tar_to_own - target_course)
                    
                else:
                    beta_tar_to_own     = np.nan
                setattr(self, f"beta_tar_to_own_{i+1}", beta_tar_to_own)
                beta_tar_to_own_list.append(beta_tar_to_own)
                
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
            dist_arr            = np.array(dist_list, dtype=float)
            collision_status    = dist_arr <= self.collision_zone_radius
            self.ship_collision = bool(np.any(collision_status))    # OUTPUT
            
            if self.ship_collision:
                self.colav_speed_reduction_factor   = 0.0   # OUTPUT
                self.colav_heading_offset_deg       = 0.0   # OUTPUT
                
                return True
            
            # Score the threat level for each target ship
            candidates = []
            for i in range(c):
                tcpa = tcpa_list[i]
                dcpa = dcpa_list[i]
                rr   = range_rate_list[i]
                
                # Colav status
                # Active if there exists target with (in order):
                # dcpa < danger_zone_radius -> If it is going to violate the safety region
                # tcpa > 0                  -> If collision will happen in the future
                # tcpa < T_lookahead        -> If collision will happen soon
                # rr   < 0 (ideally)        -> Ideally is closing in
                
                if dcpa > self.danger_zone_radius:
                    continue
                if tcpa is None or tcpa <= 0:
                    continue
                if tcpa > self.T_lookahead:
                    continue
                if rr >= 0:
                    continue
                
                # Risk score
                tcpa_score = np.clip(
                    1.0 - tcpa / self.T_lookahead,
                    0.0, 1.0
                )

                dcpa_score = np.clip(
                    1.0 - dcpa / self.danger_zone_radius,
                    0.0, 1.0
                )

                rr_score = np.clip(
                    -rr / self.own_ship_sog_max,
                    0.0, 1.0
                )

                # All coeffs must sum to 1
                tcpa_coeff  = 0.4
                dcpa_coeff  = 0.4
                rr_coeff    = 0.2
                score       = (tcpa_coeff * tcpa_score) + (dcpa_coeff * dcpa_score) + (rr_coeff * rr_score)
                candidates.append((score,i))
            
            self.colav_active   = bool(candidates) # OUTPUTS
            
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
                best_score, idx  = max(candidates, key=lambda x: x[0]) # For every tuple in candidates, looks at element 0, i.e. the score
                
                ## Prioritize one_target if multiple ship in vicinity (and not yet prioritizing one target)
                # Start tracking the target
                if len(candidates) > 1 and (not self.prioritize_one_target):
                    self.held_target_idx = idx
                    self.prioritize_one_target = True   # PRIORITIZING ONLY HAPPEN WHEN WE HAVE >1 CANDIDATES
                    self.timer = 0.0
                    
                # If prioritize one target use held_target_idx, else use idx
                use_idx         = self.held_target_idx if self.prioritize_one_target else idx
                 
                # Get the score by index list following the holding logic
                score_by_idx    = {target_idx: score for score, target_idx in candidates}
                
                # Assess the COLREG situation
                beta_own_to_tar = beta_own_to_tar_list[use_idx]
                beta_tar_to_own = beta_tar_to_own_list[use_idx]
                range_rate      = range_rate_list[use_idx]
                risk_score      = score_by_idx[use_idx]
                (encounter, role, heading_offset_deg, speed_reduction_factor) = self.COLREG_situation_assessment(beta_own_to_tar,
                                                                                                                 beta_tar_to_own,
                                                                                                                 range_rate, risk_score,
                                                                                                                 turning_direction=self.turning_direction)
                
                # Outputs prioritized target ship index
                self.ship_priority_idx  = use_idx
                
                # OUTPUTS
                self.colav_speed_reduction_factor   = speed_reduction_factor  # clamp after scaling
                self.colav_heading_offset_deg       = heading_offset_deg
                self.encounter_status               = encounter
                self.role_status                    = role
                    
                if self.prioritize_one_target:                        
                    self.timer += step_size
                    if self.timer >= self.hold_time:
                        self.prioritize_one_target = False
                        self.timer = 0.0
            
            else:
                # OUTPUTS
                self.colav_speed_reduction_factor   = 1.0
                self.colav_heading_offset_deg       = 0.0 # OUTPUT
                self.encounter_status               = "None"
                self.role_status                    = "None"
                
                # No target ship is prioritized
                self.ship_priority_idx              = -1
            
        except Exception as e:
            # IMPORTANT: do not crash host
            print(f"[CollisionAvoidanceCOLREGS] ERROR t={current_time} dt={step_size}: {type(e).__name__}: {e}")
            print(traceback.format_exc())
            
            # Freeze dynamics (keep last state/outputs)
            self.colav_speed_reduction_factor   = 1.0     # clamp after scaling
            self.colav_heading_offset_deg       = 0.0 # OUTPUT
            self.colav_active                   = False
            self.ship_collision                 = False
            self.encounter_status               = "None"
            self.role_status                    = "None"
            
        return True
            