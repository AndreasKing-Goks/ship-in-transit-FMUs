"""
Mission Manager AST Python FMU implementation.
This FMU manages a list of mission (waypoints and desired speeds) and provides the previous and next setpoints.
It includes a switching mechanism based on a radius of acceptance.
This FMU specifically designed to hande Intermediate Waypoint Sampling v2.
This FMU contains a minimum 2 setpoints to a maximum 10 setpoints,
with at max 5 intermediate waypoint betweem each waypoint.

Authors : Andreas R.G. Sitorus
Date    : Januari 2026
"""

from pythonfmu import Fmi2Causality, Fmi2Slave, Fmi2Variability, Real, Integer, Boolean
import numpy as np
import traceback


class MissionManagerAST(Fmi2Slave):
    author = "Andreas R.G. Sitorus"
    description = "Mission Manager AST (index-based, deterministic, AST compliant)"

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # Parameters
        self.ra                         = 300.0
        self.max_inter_wp               = 8
        self.max_sampled_sp             = 5
        self.scope_angle_max_deg        = 45
        self.scope_length               = 2500  # 2.5 km

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
        self.is_inside_trigger_zone     = False
        self.scope_angle_deg            = 0.0

        # Outputs
        self.prev_wp_north              = 0.0
        self.prev_wp_east               = 0.0
        self.prev_wp_speed              = 0.0

        self.next_wp_north              = 0.0
        self.next_wp_east               = 0.0
        self.next_wp_speed              = 0.0

        self.in_last_segment            = False
        
        self.reach_wp_end               = False
        
        self.request_scope_angle        = False
        
        self.successful_is_sample       = False
        

        ## Internal
        # Trajectory (the one that will be altered)
        self._traj                      = []
        self._idx                       = 0
        self._idx_in_segment            = 1
        self._traj_built                = False
        
        # Initial route trajectory
        self._segment_idx               = 1     # Means segment No. 1
        self._traj_base                 = []    # Original Planned Trajectory, not altered
        self._i_sp_list                 = []    # Intermediate waypoint lists
        self._p_pj_list                 = []    # Intermediate waypoint projection to the route segment point list
        self._segment_switch            = False
        self._in_last_segment           = False
        self._init_i_setpoint_sampler   = True
        self._actual_is_sampling_count  = 0

        # Registration
        self.register_variable(Real("ra", causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))
        self.register_variable(Integer("max_inter_wp", causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))
        self.register_variable(Integer("max_sampled_sp", causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))
        self.register_variable(Real("scope_angle_max_deg", causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))
        self.register_variable(Real("scope_length", causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))
        
        self.register_variable(Real("north", causality=Fmi2Causality.input))
        self.register_variable(Real("east", causality=Fmi2Causality.input))
        self.register_variable(Boolean("is_inside_trigger_zone", causality=Fmi2Causality.input))
        self.register_variable(Real("scope_angle_deg", causality=Fmi2Causality.input))

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

        self.register_variable(Boolean("in_last_segment", causality=Fmi2Causality.output))
        
        self.register_variable(Boolean("reach_wp_end", causality=Fmi2Causality.output))
        
        self.register_variable(Boolean("request_scope_angle", causality=Fmi2Causality.output))
        
        self.register_variable(Boolean("successful_is_sample", causality=Fmi2Causality.output))

    def _valid_triplet(self, n, e, s) -> bool:
        return np.isfinite(n) and np.isfinite(e) and np.isfinite(s)

    def _build_trajectory(self):
        traj = []

        # Start
        if self._valid_triplet(self.wp_start_north, self.wp_start_east, self.wp_start_speed):
            traj.append((float(self.wp_start_north), float(self.wp_start_east), float(self.wp_start_speed)))

        # Intermediate (1..max_inter_wp)
        m = int(self.max_inter_wp)
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
        self._traj_base                 = traj.copy()              # Original Planned Trajectory, not altered
        self._idx                       = 0
        self._idx_in_segment            = 1
        self._segment_idx               = 1
        self._max_segment_idx           = len(self._traj_base) - 1
        self._traj_built                = True
        self._segment_switch            = False
        self._init_i_setpoint_sampler   = True
        self._in_last_segment           = False
        self._actual_is_sampling_count  = 0
        self._i_sp_list                 = []
        self._p_pj_list                 = []

        # Initialize outputs if we have at least 2 points
        if len(self._traj) >= 2:
            prev_setpoint = self._traj[0]
            next_setpoint = self._traj[1]
            self.prev_wp_north, self.prev_wp_east, self.prev_wp_speed = prev_setpoint
            self.next_wp_north, self.next_wp_east, self.next_wp_speed = next_setpoint
            self.in_last_segment = False
        else:
            # Not enough points
            self.prev_wp_north = self.prev_wp_east = self.prev_wp_speed = 0.0
            self.next_wp_north = self.next_wp_east = self.next_wp_speed = 0.0
            self.in_last_segment = False

    def _dist2_to_next(self) -> float:
        n2, e2, _ = self._traj[self._idx + 1]
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
        
    def _get_i_wp_parameters(self, i_setpoint, base_setpoint, beta, segment_length):
        # Unpack intermediate setpoint
        n_i_wp               = i_setpoint[0]
        e_i_wp               = i_setpoint[1]
        
        # Unpack base setpoint
        base_north           = base_setpoint[0]
        base_east            = base_setpoint[1]
        
        # Base to IW segment line components' length
        d_base_to_i_wp_north = n_i_wp - base_north
        d_base_to_i_wp_east  = e_i_wp - base_east
        
        # Base to IW segment line length
        base_i_wp_length     = np.hypot(d_base_to_i_wp_north, d_base_to_i_wp_east)
        
        # Angle between the North axis to the IW segment line
        beta_i_wp            = np.atan2(d_base_to_i_wp_east, d_base_to_i_wp_north)
        
        # Angle between the route segment line to the IW segment line
        psi                  = beta_i_wp - beta
        
        # Traversed segment Length
        traversed_segment_length    = base_i_wp_length * np.cos(psi)
        segment_arm_length          = base_i_wp_length * np.sin(psi)
        untraversed_segment_length  = segment_length - traversed_segment_length
        
        # Projection point coordinate to the segment route
        p_n                     = base_north + traversed_segment_length * np.cos(beta)
        p_e                     = base_east  + traversed_segment_length * np.sin(beta)
        p_pj_coord              = (p_n, p_e)
        
        return (traversed_segment_length,
                untraversed_segment_length,
                segment_arm_length,
                p_pj_coord)
        
    def _scope_next_i_wp(self, scope_angle_deg, scope_length, beta, prev_wp_north, prev_wp_east):
        # Convert scope angle deg to rad
        psi         = np.deg2rad(float(scope_angle_deg))
        
        # Get angle form the North axist to the IW line
        beta_i_wp   = beta + psi
        
        # Get the d_north and d_east
        d_north     = scope_length * np.cos(beta_i_wp)
        d_east      = scope_length * np.sin(beta_i_wp)
        
        # Get the next waypoint
        next_wp_north = prev_wp_north + float(d_north)
        next_wp_east  = prev_wp_east  + float(d_east)
        
        return (next_wp_north,
                next_wp_east)
        
    def _insert_i_sp_to_traj(self, i_setpoint):
        # Get the route segment base_setpoint index in self._traj
        base_setpoint       = self._traj_base[self._segment_idx - 1]
        base_setpoint_idx   = self._traj.index(base_setpoint)
        
        # Identify the entry index for the given intermediate way point
        entry_idx           = base_setpoint_idx + self._idx_in_segment
        
        # Insert the intermediate waypoint to traj
        self._traj.insert(entry_idx, i_setpoint)
        
        # Increment the segment's waypoint index
        self._idx_in_segment += 1
        
    def _do_switch_segment(self, untraversed_segment_length, i_setpoint, p_pj_coord):
        """
        Decide what to do with the proposed intermediate setpoint.

        Returns
        -------
        decision : str
            One of:
            - "insert_now"
            - "postpone_to_next_segment"
            - "inserted_in_last_segment"
        """
        # Default: no postponement, insert in current segment
        if untraversed_segment_length >= self.scope_length:
            return "insert_now"

        # Proposed IS is too close to current segment head
        if self._segment_idx < self._max_segment_idx:
            self._segment_idx += 1
            self._idx_in_segment = 1
            self._segment_switch = True
            return "postpone_to_next_segment"

        # Last segment: cannot postpone further, so insert here
        self._i_sp_list.append(i_setpoint)
        self._p_pj_list.append(p_pj_coord)
        self._insert_i_sp_to_traj(i_setpoint)
        self._in_last_segment = True
        
        return "inserted_in_last_segment"
    
    def _place_zeroeth_i_sp(self):
        # Set the first ship position that triggers IS sampler as the 0th IS
        i_setpoint  = (float(self.north), float(self.east), self.next_wp_speed)   # IS's desired speed follows the segment route
        
        # Get the route segment parameters
        base_setpoint, head_setpoint, segment_length, beta = self._get_route_segment_parameters()
        
        # Get the IW parameters
        _, untraversed_segment_length, _, p_pj_coord  = self._get_i_wp_parameters(i_setpoint, base_setpoint, beta, segment_length)
        
        # Get the decision for the segment switch
        decision = self._do_switch_segment(untraversed_segment_length, i_setpoint, p_pj_coord)

        # In the last segment, add the last setpoint immediately
        if decision == "insert_now":
            self._i_sp_list.append(i_setpoint)
            self._p_pj_list.append(p_pj_coord)
            self._insert_i_sp_to_traj(i_setpoint)
    
    def _get_intermediate_setpoint(self, scope_angle_deg):        
        # Get route segment parameters
        base_setpoint, head_setpoint, segment_length, beta = self._get_route_segment_parameters()
        
        # Get the previous waypoint as a base to scope the next intermediate waypoint
        # Special case, when IW sample request happened during the segment switch,
        # Use the base of the next segment as the base for next IW scoping
        if self._segment_switch:
            prev_n_i_wp, prev_e_i_wp, _  = base_setpoint
            self._segment_switch         = False
        # Else, use previous IW as the base for the next IW scoping
        else:
            base_setpoint_idx            = self._traj.index(base_setpoint)
            prev_wp_idx                  = base_setpoint_idx + self._idx_in_segment - 1 # -1 due to previous increment in _insert_i_sp_to_traj
            prev_n_i_wp, prev_e_i_wp, _  = self._traj[prev_wp_idx]
        
        # Scope the next intermediate waypoint given the input scope angle
        n_i_wp, e_i_wp  = self._scope_next_i_wp(scope_angle_deg, self.scope_length, beta, prev_n_i_wp, prev_e_i_wp)
        i_setpoint = (n_i_wp, e_i_wp, self.next_wp_speed)
        
        # Get the IW parameters
        _, untraversed_segment_length, _, p_pj_coord = self._get_i_wp_parameters(i_setpoint, base_setpoint, beta, segment_length)
        
        # Get the decision for the segment switch
        decision = self._do_switch_segment(untraversed_segment_length, i_setpoint, p_pj_coord)

        # In the last segment, add the last setpoint immediately
        if decision == "insert_now":
            self._i_sp_list.append(i_setpoint)
            self._p_pj_list.append(p_pj_coord)
            self._insert_i_sp_to_traj(i_setpoint)

        return decision
            
    def do_step(self, current_time: float, step_size: float) -> bool:
        try:
            # Build once (or rebuild if you want: detect changes; keeping it simple)
            if not self._traj_built:
                self._build_trajectory()

            # If not enough waypoints, nothing to do
            if len(self._traj) < 2:
                return True
            
            # If we got intermediate setpoint sampling request, alter the 
            # self._traj. The self._traj is responsible for handling the switching
            # During IS sampling, speed is conditioned to follow the next_wp_speed 
            # of its current segment
            
            # Get squared radius of acceptance
            ra2 = float(self.ra) * float(self.ra)
            
            ########################################## IS SAMPLE ##########################################
            # Initial output value
            self.successful_is_sample   = False
            
            # List of triggers
            entering_roa                = self._dist2_to_next() <= ra2      # Ship enters RoA
            next_sp_is_intermediate     = self._traj[self._idx + 1] not in self._traj_base
            
            # List of conditions. self.is_inside_trigger_zone <-- INPUT
            init_is_sampling            = self.is_inside_trigger_zone and self._init_i_setpoint_sampler
            is_sampling                 = self.is_inside_trigger_zone and entering_roa and next_sp_is_intermediate and (not self._segment_switch)
            is_sampling_after_switch    = self.is_inside_trigger_zone and entering_roa and (not next_sp_is_intermediate) and self._segment_switch
            within_sampling_bound       = self._actual_is_sampling_count < self.max_sampled_sp
            
            # Place the zeroeth IS ONCE if the IS sampler is active and
            # if inside the trigger zone, prepare for sampling new intermediate setpoint
            if init_is_sampling and within_sampling_bound:
                # First enter then request scope angle while not doing segment switch
                if (not self.request_scope_angle):
                    # Place the IS 0   
                    self._place_zeroeth_i_sp()             
                    
                    # Request scope angle
                    self.request_scope_angle = True
                    
                    return True
                
                # Scope angle has been requested while not doing segment switch
                elif self.request_scope_angle:
                    # Sample IS and store it to self._traj
                    decision = self._get_intermediate_setpoint(scope_angle_deg=self.scope_angle_deg)       # self.scope_angle_deg <-- INPUT
                    
                    if decision == "insert_now" or decision == "inserted_in_last_segment":
                        # Successful IS sampling
                        self.successful_is_sample = True
                        
                        # Increment the IS sampling count
                        self._actual_is_sampling_count += 1
                        
                        # Turn off the the request scope angle flag
                        self.request_scope_angle = False
                        
                        # Increment the sampler index
                        self._idx += 1
                        
                        # Update prev/next using new index
                        prev_setpoint = self._traj[self._idx]
                        next_setpoint = self._traj[self._idx + 1] if (self._idx + 1) < len(self._traj) else self._traj[-1]

                        self.prev_wp_north, self.prev_wp_east, self.prev_wp_speed = prev_setpoint
                        self.next_wp_north, self.next_wp_east, self.next_wp_speed = next_setpoint
                    
                        return True
                    
                    elif decision == "postpone_to_next_segment":
                        self.successful_is_sample = False
                
            elif is_sampling and within_sampling_bound: # Also check if the next waypoint is not inside
                # First enter then request scope angle while not doing segment switch
                if (not self.request_scope_angle):
                    # Request scope angle
                    self.request_scope_angle = True

                    return True
                
                # Scope angle has been requested while not doing segment switch
                elif self.request_scope_angle:
                    # Sample IS and store it to self._traj
                    decision = self._get_intermediate_setpoint(scope_angle_deg=self.scope_angle_deg)       # self.scope_angle_deg <-- INPUT
                    
                    if decision == "insert_now" or decision == "inserted_in_last_segment":
                        # Successful IS sampling
                        self.successful_is_sample = True
                        
                        # Increment the IS sampling count
                        self._actual_is_sampling_count += 1
                        
                        # Turn off the the request scope angle flag
                        self.request_scope_angle = False
                        
                        # Increment the sampler index
                        self._idx += 1
                        
                        # Update prev/next using new index
                        prev_setpoint = self._traj[self._idx]
                        next_setpoint = self._traj[self._idx + 1] if (self._idx + 1) < len(self._traj) else self._traj[-1]

                        self.prev_wp_north, self.prev_wp_east, self.prev_wp_speed = prev_setpoint
                        self.next_wp_north, self.next_wp_east, self.next_wp_speed = next_setpoint
                    
                        return True
                    
                    elif decision == "postpone_to_next_segment":
                        self.successful_is_sample = False
            
            elif is_sampling_after_switch and within_sampling_bound:
                # First enter then request scope angle while not doing segment switch
                if (not self.request_scope_angle):
                    # Request scope angle
                    self.request_scope_angle = True

                    return True
                
                # Scope angle has been requested while not doing segment switch
                elif self.request_scope_angle:
                    # Sample IS and store it to self._traj
                    decision = self._get_intermediate_setpoint(scope_angle_deg=self.scope_angle_deg)       # self.scope_angle_deg <-- INPUT
                    
                    if decision == "insert_now" or decision == "inserted_in_last_segment":
                        # Successful IS sampling
                        self.successful_is_sample = True
                        
                        # Increment the IS sampling count
                        self._actual_is_sampling_count += 1
                        
                        # Turn off the the request scope angle flag
                        self.request_scope_angle = False
                        
                        # Increment the sampler index
                        self._idx += 1
                        
                        # Update prev/next using new index
                        prev_setpoint = self._traj[self._idx]
                        next_setpoint = self._traj[self._idx + 1] if (self._idx + 1) < len(self._traj) else self._traj[-1]

                        self.prev_wp_north, self.prev_wp_east, self.prev_wp_speed = prev_setpoint
                        self.next_wp_north, self.next_wp_east, self.next_wp_speed = next_setpoint
                    
                        return True
                    
                    elif decision == "postpone_to_next_segment":
                        self.successful_is_sample = False
                
            #######################################################################################################

            # If we're already on last segment, latch and keep outputs fixed
            # while keep checking if the ship reaches the last two end point
            if self._idx >= len(self._traj) - 2:
                self.in_last_segment = True
                self.reach_wp_end    = False if self.reach_wp_end is False else True # If it's already switched to True, stay True.
                
                # Check if the ship reaches the end point (three-quarter way inside the waypoint's RoA)
                if entering_roa:
                    self.in_last_segment = True
                    self.reach_wp_end    = True
                    
                    # Set the speed set point to 0 (Stopping)
                    self.prev_wp_speed   = 0.0
                    self.next_wp_speed   = 0.0
                           
                return True
            
            if entering_roa:
                self._idx += 1

                # Update prev/next using new index
                prev_setpoint = self._traj[self._idx]
                next_setpoint = self._traj[self._idx + 1] if (self._idx + 1) < len(self._traj) else self._traj[-1]

                self.prev_wp_north, self.prev_wp_east, self.prev_wp_speed = prev_setpoint
                self.next_wp_north, self.next_wp_east, self.next_wp_speed = next_setpoint
            
        except Exception as e:
            # Keep host alive; do not crash co-sim
            print(f"[MissionManager] Exception t={current_time}, dt={step_size}: {type(e).__name__}: {e}")
            print(traceback.format_exc())

            # Freeze dynamics safely (keep last state/outputs)
            self.prev_wp_north      = 0.0
            self.prev_wp_east       = 0.0
            self.prev_wp_speed      = 0.0

            self.next_wp_north      = 0.0
            self.next_wp_east       = 0.0
            self.next_wp_speed      = 0.0
            
            self.in_last_segment    = False
            
            self.reach_wp_end       = False
            
        return True

                
                