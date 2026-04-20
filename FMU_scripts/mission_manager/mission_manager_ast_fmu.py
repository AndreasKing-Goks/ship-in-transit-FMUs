"""
Mission Manager FMU for EB.ASTv2. 
This FMU manages waypoint progression using previous and next setpoints, 
supports zeroeth and sampled intermediate waypoint insertion during time activation, 
and requests external scope-angle inputs to generate new intermediate waypoints 
during runtime. Can only holds 8 intermediate waypoints at max.

Authors : Andreas R.G. Sitorus
Date    : April 2026
"""

from pythonfmu import Fmi2Causality, Fmi2Slave, Fmi2Variability, Real, Integer, Boolean, String
import numpy as np
import traceback


class MissionManagerAST(Fmi2Slave):
    author = "Andreas R.G. Sitorus"
    description = "Mission Manager for single-segment trajectory-intermediate waypoint sampling"

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # Parameters
        self.ra                         = 300.0
        self.time_to_start_iw_sampler   = 0.0
        self.max_inner_wp               = 8
        self.max_sampled_inter_wp       = 8
        self.scope_angle_max_deg        = 45

        # Waypoints (parameters)
        self.wp_start_north             = 0.0
        self.wp_start_east              = 0.0
        self.wp_start_speed             = 0.0

        for i in range(1, 9):
            setattr(self, f"wp_{i}_north", 0.0)
            setattr(self, f"wp_{i}_east",  0.0)
            setattr(self, f"wp_{i}_speed", 0.0)

        self.wp_end_north               = 0.0
        self.wp_end_east                = 0.0
        self.wp_end_speed               = 0.0

        # Inputs
        self.north                      = 0.0
        self.east                       = 0.0
        self.scope_angle_deg            = 0.0
        self.scope_length               = 2500  # 2.5 km

        # Outputs
        self.prev_wp_north              = 0.0
        self.prev_wp_east               = 0.0
        self.prev_wp_speed              = 0.0
        self.next_wp_north              = 0.0
        self.next_wp_east               = 0.0
        self.next_wp_speed              = 0.0
        self.last_segment_active        = False
        self.reach_wp_end               = False
        self.request_captain_intent        = False
        self.insert_wp_now              = False
        self.idx                        = 0
        self.next_wp_proj_north         = 0.0
        self.next_wp_proj_east          = 0.0
            
        # Debug
        self.messages                   = "-"

        ## Internal
        # Base trajectory
        self._traj                      = []
        self._traj_built                = False
        
        # For Intermediate waypoint sampling
        self._traj_base                 = []
        self._idx_in_segment            = 1
        self._inter_wp_list             = []
        self._inter_wp_proj_list        = []
        self._accepted_sampled_inter_wp = 0
        self._segment_idx               = 1
        self._zeroeth_iw_inserted       = False
        self._wait_first_captain_intent = False
        self._handle_pending_IW_request = False
        
        # Registration
        self.register_variable(Real("ra", causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))
        self.register_variable(Real("time_to_start_iw_sampler", causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))
        self.register_variable(Integer("max_inner_wp", causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))
        self.register_variable(Integer("max_sampled_inter_wp", causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))
        self.register_variable(Real("scope_angle_max_deg", causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))

        self.register_variable(Real("north", causality=Fmi2Causality.input))
        self.register_variable(Real("east", causality=Fmi2Causality.input))
        self.register_variable(Real("scope_angle_deg", causality=Fmi2Causality.input))
        self.register_variable(Real("scope_length", causality=Fmi2Causality.input))

        self.register_variable(Real("wp_start_north", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("wp_start_east",  causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("wp_start_speed", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))

        for i in range(1, 9):
            self.register_variable(Real(f"wp_{i}_north", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
            self.register_variable(Real(f"wp_{i}_east",  causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
            self.register_variable(Real(f"wp_{i}_speed", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))

        self.register_variable(Real("wp_end_north", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("wp_end_east",  causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("wp_end_speed", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))

        self.register_variable(Real("prev_wp_north", causality=Fmi2Causality.output))
        self.register_variable(Real("prev_wp_east",  causality=Fmi2Causality.output))
        self.register_variable(Real("prev_wp_speed", causality=Fmi2Causality.output))
        self.register_variable(Real("next_wp_north", causality=Fmi2Causality.output))
        self.register_variable(Real("next_wp_east",  causality=Fmi2Causality.output))
        self.register_variable(Real("next_wp_speed", causality=Fmi2Causality.output))
        self.register_variable(Boolean("last_segment_active", causality=Fmi2Causality.output))
        self.register_variable(Boolean("reach_wp_end", causality=Fmi2Causality.output))
        self.register_variable(Boolean("request_captain_intent", causality=Fmi2Causality.output))
        self.register_variable(Boolean("insert_wp_now", causality=Fmi2Causality.output))
        self.register_variable(Integer("idx", causality=Fmi2Causality.output))
        self.register_variable(Real("next_wp_proj_north", causality=Fmi2Causality.output))
        self.register_variable(Real("next_wp_proj_east", causality=Fmi2Causality.output))
        
        # Debug
        self.register_variable(String("messages", causality=Fmi2Causality.output))

    def _valid_triplet(self, n, e, s) -> bool:
        return np.isfinite(n) and np.isfinite(e) and np.isfinite(s)

    def _build_trajectory(self):
        traj = []

        # Start
        if self._valid_triplet(self.wp_start_north, self.wp_start_east, self.wp_start_speed):
            traj.append((float(self.wp_start_north), float(self.wp_start_east), float(self.wp_start_speed)))

        # Intermediate (1..max_inner_wp)
        m = int(self.max_inner_wp)
        m = max(0, min(m, 8))
        for i in range(1, m + 1):
            n = getattr(self, f"wp_{i}_north")
            e = getattr(self, f"wp_{i}_east")
            s = getattr(self, f"wp_{i}_speed")
            if self._valid_triplet(n, e, s):
                traj.append((float(n), float(e), float(s)))

        # End
        if self._valid_triplet(self.wp_end_north, self.wp_end_east, self.wp_end_speed):
            traj.append((float(self.wp_end_north), float(self.wp_end_east), float(self.wp_end_speed)))

        self._traj                      = traj.copy()
        self.idx                       = 0
        self._traj_built                = True
        
        self._traj_base                 = traj.copy()
        self._idx_in_segment            = 1
        self._inter_wp_list             = []
        self._inter_wp_proj_list        = []
        self._accepted_sampled_inter_wp = 0
        self._segment_idx               = 1
        self._max_segment_idx           = len(traj)
        self._zeroeth_iw_inserted       = False
        self._wait_first_captain_intent = False
        self._handle_pending_IW_request = False

        # Initialize outputs if we have at least 2 points
        if len(self._traj) >= 2:
            p = self._traj[0]
            q = self._traj[1]
            self.prev_wp_north, self.prev_wp_east, self.prev_wp_speed = p
            self.next_wp_north, self.next_wp_east, self.next_wp_speed = q
            self.last_segment_active = False
        else:
            # Not enough points
            self.prev_wp_north = self.prev_wp_east = self.prev_wp_speed = 0.0
            self.next_wp_north = self.next_wp_east = self.next_wp_speed = 0.0
            self.last_segment_active = False

    def _dist2_to_next(self) -> float:
        if (self.idx + 1) >= len(self._traj):
            return np.inf
        n2, e2, _ = self._traj[self.idx + 1]
        dn = n2 - float(self.north)
        de = e2 - float(self.east)
        return dn * dn + de * de
    
    def _get_route_segment_parameters(self):
        # Get the base and the head setpoint
        base_setpoint   = self._traj_base[self._segment_idx - 1]
        head_setpoint   = self._traj_base[self._segment_idx]
        
        # Get the route segment base coordinate
        base_north      = base_setpoint[0]
        base_east       = base_setpoint[1]
        
        # Get the route segment head coordinate
        head_north      = head_setpoint[0]
        head_east       = head_setpoint[1]
        
        # Route segment components' length
        d_segment_north = head_north - base_north
        d_segment_east  = head_east  - base_east
        
        # Compute the route segment length and angle from the North axis to the route segment line
        segment_length  = np.hypot(d_segment_north, d_segment_east)
        beta            = np.atan2(d_segment_east, d_segment_north)
        
        return base_setpoint, head_setpoint, segment_length, beta
        
    def _get_inter_wp_parameters(self, inter_wp, base_setpoint, beta, segment_length):
        # Unpack intermediate setpoint
        n_inter_wp                  = inter_wp[0]
        e_inter_wp                  = inter_wp[1]
        
        # Unpack base setpoint
        base_north                  = base_setpoint[0]
        base_east                   = base_setpoint[1]
        
        # Base to IW segment line components' length
        d_base_to_inter_wp_north    = n_inter_wp - base_north
        d_base_to_inter_wp_east     = e_inter_wp - base_east
        
        # Base to IW segment line length
        base_inter_wp_length        = np.hypot(d_base_to_inter_wp_north, d_base_to_inter_wp_east)
        
        # Angle between the North axis to the IW segment line
        beta_inter_wp               = np.atan2(d_base_to_inter_wp_east, d_base_to_inter_wp_north)
        
        # Angle between the route segment line to the IW segment line
        psi                         = beta_inter_wp - beta
        
        # Traversed segment Length
        traversed_segment_length    = base_inter_wp_length * np.cos(psi)
        segment_arm_length          = base_inter_wp_length * np.sin(psi)
        untraversed_segment_length  = segment_length - traversed_segment_length
        
        # Projection point coordinate to the segment route
        p_n                         = base_north + traversed_segment_length * np.cos(beta)
        p_e                         = base_east  + traversed_segment_length * np.sin(beta)
        inter_wp_proj               = (p_n, p_e)
        
        return (traversed_segment_length,
                untraversed_segment_length,
                segment_arm_length,
                inter_wp_proj)
        
    def _scope_next_inter_wp(self, scope_angle_deg, scope_length, beta, prev_wp_north, prev_wp_east):
        # Convert scope angle deg to rad
        psi             = np.deg2rad(float(scope_angle_deg))
        
        # Get angle form the North axist to the IW line
        beta_inter_wp   = beta + psi
        
        # Get the d_north and d_east
        d_north         = scope_length * np.cos(beta_inter_wp)
        d_east          = scope_length * np.sin(beta_inter_wp)
        
        # Get the next waypoint
        next_wp_north   = prev_wp_north + float(d_north)
        next_wp_east    = prev_wp_east  + float(d_east)
        
        return (next_wp_north,
                next_wp_east)
        
    def _insert_inter_wp_to_traj(self, inter_wp):
        # Get the route segment base_setpoint index in self._traj
        base_setpoint       = self._traj_base[self._segment_idx - 1]
        base_setpoint_idx   = self._traj.index(base_setpoint)
        
        # Identify the entry index for the given intermediate way point
        entry_idx           = base_setpoint_idx + self._idx_in_segment
        
        # Insert the intermediate waypoint to traj
        self._traj.insert(entry_idx, inter_wp)
        
        # Increment the segment's waypoint index
        self._idx_in_segment += 1
    
    def _get_zeroeth_intermediate_waypoint(self):
        # Set the first ship position that triggers IS sampler as the 0th IS
        inter_wp  = (float(self.north), float(self.east), self.next_wp_speed)   # IS's desired speed follows the segment route
        
        # Get the route segment parameters
        base_setpoint, head_setpoint, segment_length, beta = self._get_route_segment_parameters()
        
        # Get the IW parameters
        _, _, _, inter_wp_proj  = self._get_inter_wp_parameters(inter_wp, base_setpoint, beta, segment_length)
        
        # Insert the zeroeth waypoint to and its projection to its respected lists
        self._inter_wp_list.append(inter_wp)
        self._inter_wp_proj_list.append(inter_wp_proj)
        self._insert_inter_wp_to_traj(inter_wp)
        
        # Store output
        self.next_wp_proj_north, self.next_wp_proj_east = self._inter_wp_proj_list[-1]
    
    def _get_intermediate_waypoint(self, scope_angle_deg, scope_length):        
        # Get route segment parameters
        base_setpoint, head_setpoint, segment_length, beta = self._get_route_segment_parameters()
        
        # Get the previous waypoint as a base to scope the next intermediate waypoint
        base_setpoint_idx                    = self._traj.index(base_setpoint)
        prev_wp_idx                          = base_setpoint_idx + self._idx_in_segment - 1 # -1 due to previous increment in _insert_inter_wp_to_traj
        prev_n_inter_wp, prev_e_inter_wp, _  = self._traj[prev_wp_idx]
        
        # Scope the next intermediate waypoint given the input scope angle
        n_inter_wp, e_inter_wp  = self._scope_next_inter_wp(scope_angle_deg, scope_length, beta, prev_n_inter_wp, prev_e_inter_wp)
        inter_wp = (n_inter_wp, e_inter_wp, self.next_wp_speed)
        
        # Get the IW parameters
        _, untraversed_segment_length, _, inter_wp_proj = self._get_inter_wp_parameters(inter_wp, base_setpoint, beta, segment_length)
        
        # Determine if switching segment or not
        segment_switch = self._do_segment_switch(untraversed_segment_length, scope_length)

        if not segment_switch:
            # Insert the zeroeth waypoint to and its projection to its respected lists
            self._inter_wp_list.append(inter_wp)
            self._inter_wp_proj_list.append(inter_wp_proj)
            self._insert_inter_wp_to_traj(inter_wp)
            
            # Store output
            self.next_wp_proj_north, self.next_wp_proj_east = self._inter_wp_proj_list[-1]
        
        return segment_switch
        
    def _do_segment_switch(self, untraversed_segment_length, scope_length):
        """
        Decide what to do with the proposed intermediate setpoint.
        If the scope length is shorter than the untraversed segment length, do segment switch
        """
        # If no segment switch, return False
        if untraversed_segment_length >= scope_length:
            return False
        
        # Else return True
        return True
    
    def _advance_traj(self):
        self.idx += 1
        
    def _reverse_traj(self):
        self.idx -= 1

    def _get_prev_and_next_waypoint(self):
        # Update prev/next using new index
        p = self._traj[self.idx] if (self.idx + 1) < len(self._traj) else self._traj[-2]
        q = self._traj[self.idx + 1] if (self.idx + 1) < len(self._traj) else self._traj[-1]

        self.prev_wp_north, self.prev_wp_east, self.prev_wp_speed = p
        self.next_wp_north, self.next_wp_east, self.next_wp_speed = q        
        
    def do_step(self, current_time: float, step_size: float) -> bool:
        try:
            # Build once (or rebuild if you want: detect changes; keeping it simple)
            if not self._traj_built:
                self._build_trajectory()

            # If not enough waypoints, nothing to do
            if len(self._traj) < 2:
                return True
            
            # Check if the ship enters radius of acceptance
            ra2                 = float(self.ra) * float(self.ra)
            entering_ra         = self._dist2_to_next() <= ra2          # Ship enters RoA
            
            # Next-point status
            next_wp_exists = (self.idx + 1) < len(self._traj)
            next_wp_is_end = next_wp_exists and (self._traj[self.idx + 1] == self._traj_base[-1])
            next_wp_is_inter = next_wp_exists and (self._traj[self.idx + 1] not in self._traj_base)

            # Last segment flag
            on_final_leg                = (self.idx == len(self._traj) - 2)
            self.last_segment_active    = on_final_leg
            
            # List of triggers
            within_sampling_max_count   = self._accepted_sampled_inter_wp < self.max_sampled_inter_wp
            iw_sampler_active           = current_time >= self.time_to_start_iw_sampler
            place_zeroeth_inter_wp      = (within_sampling_max_count 
                                           and iw_sampler_active
                                           and not self._zeroeth_iw_inserted)
            enable_inter_wp_sampling    = (within_sampling_max_count 
                                           and iw_sampler_active
                                           and self._zeroeth_iw_inserted
                                           and entering_ra 
                                           and next_wp_is_inter
                                           )

            # Output for external function
            messages            = ""
            self.insert_wp_now  = False
            
            # ======================================================================================
            # First Condition: Handle the segment switching
            # ======================================================================================
            if entering_ra and self._handle_pending_IW_request:
                # Advance the base route segment index and reset
                if self._segment_idx < self._max_segment_idx:
                    self._segment_idx          += 1
                    self._idx_in_segment        = 1
                    self.request_captain_intent    = True
                else:
                    self.request_captain_intent    = False
                
                # Reset the handle pending IW request flag
                self._handle_pending_IW_request = False
                
                # Debug
                messages        += f"Scope angle request at base waypoint ({self.next_wp_north:.1f},{self.next_wp_east:.1f}) -> "
                self.messages    = messages
            
            # ======================================================================================
            # Second Condition: Handle the captain intent request
            # ======================================================================================
            if self.request_captain_intent:
                # Compute the intermediate waypoint
                segment_switch = self._get_intermediate_waypoint(scope_angle_deg=self.scope_angle_deg, scope_length=self.scope_length)
                
                # Advance the trajectory and temporarily pending the captain intent request while awaiting for the next waypoint RoA visit
                if segment_switch:
                    self._advance_traj()
                    self._get_prev_and_next_waypoint()
                    
                    self._handle_pending_IW_request = True
                    self.request_captain_intent     = False
                    
                    # Debug
                    messages                        = f"Segment switch from segment {self._segment_idx - 1} to segment {self._segment_idx}"
                    self.messages                   = messages
                    
                    return True
                
                # Condition when then ship first enter the trigger zone after getting the zeroeth intermediate waypoint
                if self._wait_first_captain_intent:    
                    # Advance the trajectory
                    self._advance_traj()
                    self._get_prev_and_next_waypoint()
                    
                    # Finished waiting first captain intent
                    self._wait_first_captain_intent  = False
                    
                    sub_messages  = f"First captain intent using scope angle of {self.scope_angle_deg:.1f} degrees and scope length of {self.scope_length}"
                else:
                    # Advance the trajectory
                    self._advance_traj()
                    self._get_prev_and_next_waypoint()
                    sub_messages  = f"Second and Onwards captain intent using scope angle of {self.scope_angle_deg:.1f} degrees and scope length of {self.scope_length}"
                    
                # Then turn off the captain intent request after receiving it
                self.request_captain_intent        = False
                
                # Increment the sampling count after the sampling
                self._accepted_sampled_inter_wp += 1
                
                # Set the internal function to insert the wp now for recording
                self.insert_wp_now              = True
                
                # Debug
                messages        += f"Captain intent request handling: {sub_messages}"
                self.messages    = messages
                
                return True
                    
            # ======================================================================================
            # Third Condition: Zeroeth intermediate waypoint placement + request first captain intent
            # ======================================================================================
            if place_zeroeth_inter_wp:
                # Update the actual segment index. Inside WP trigger does not immediately triggers at the first waypoint
                prev_wp = (self.prev_wp_north, self.prev_wp_east, self.prev_wp_speed)
                self._segment_idx = self._traj_base.index(prev_wp) + 1
                
                # Place the zeroeth intermediate waypoint
                self._get_zeroeth_intermediate_waypoint()
                
                # Stay at the current trajectory
                self._get_prev_and_next_waypoint()
                
                # Request the scope angle for the next step
                self.request_captain_intent     = True
                
                # Set the necessary flags
                self._zeroeth_iw_inserted       = True
                self._wait_first_captain_intent = True
                
                # Set the internal function to insert the wp now for recording
                self.insert_wp_now              = True
                
                # Debug
                messages       += f"Zeroeth IW Placement at ({self.next_wp_north:.1f},{self.next_wp_east:.1f}) + First captain intent request."
                self.messages   = messages

                return True
            
            # ======================================================================================
            # Fourth Condition: Request second and onwards captain intent
            # ======================================================================================
            elif enable_inter_wp_sampling:
                # Request the scope angle for the next step
                self.request_captain_intent    = True
                
                # Debug
                messages       += f"Entering waypoint ({self.next_wp_north:.1f},{self.next_wp_east:.1f}) RoA -> Second and onwards captain intent request"
                self.messages   = messages
                
                return True
            
            # ======================================================================================
            # Fifth Condition: End Condition
            # ======================================================================================
            if entering_ra and next_wp_is_end:
                self.reach_wp_end           = True
                self.prev_wp_speed          = 0.0
                self.next_wp_speed          = 0.0
                
                # Debug
                messages       += f"Reaching the final waypoint ({self.next_wp_north:.1f},{self.next_wp_east:.1f})"
                self.messages   = messages
                
                return True
                
            # ======================================================================================
            # Sixth Condition: Normal Progression
            # ======================================================================================
            if entering_ra:
                # Advance the trajectory
                self._advance_traj()
                self._get_prev_and_next_waypoint()
                
                # Debug
                messages       += f"Entering waypoint ({self.prev_wp_north:.1f},{self.prev_wp_east:.1f}) RoA"
                self.messages   = messages
                
                return True
            
            # Debug
            messages       += f"Try to converge to segment ({self.prev_wp_north:.1f},{self.prev_wp_east:.1f})->({self.next_wp_north:.1f},{self.next_wp_east:.1f})"
            self.messages   = messages
                
            return True
            
        except Exception as e:
            # Keep host alive; do not crash co-sim
            print(f"[MissionManager] Exception t={current_time}, dt={step_size}: {type(e).__name__}: {e}")
            print(traceback.format_exc())

            # Freeze dynamics safely (keep last state/outputs)
            self.last_segment_active    = False
            self.reach_wp_end           = False
            self.request_captain_intent = False
            self.insert_wp_now          = False
            
        return True

                
                