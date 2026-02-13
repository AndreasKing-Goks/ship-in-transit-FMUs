"""
Mission Manager Python FMU implementation.
This FMU manages a list of mission (waypoints and desired speeds) and provides the previous and next setpoints.
It includes a switching mechanism based on a radius of acceptance.
This FMU contains a minimum 2 setpoints to a maximum 10 setpoints.

Authors : Andreas R.G. Sitorus
Date    : Januari 2026
"""

from pythonfmu import Fmi2Causality, Fmi2Slave, Fmi2Variability, Real, Integer, Boolean
import numpy as np
import traceback


class MissionManager(Fmi2Slave):
    author = "Andreas R.G. Sitorus"
    description = "Mission Manager (index-based, deterministic)"

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # Parameters
        self.ra = 300.0
        self.max_inter_wp = 8

        # Waypoints (parameters)
        self.wp_start_north = 0.0
        self.wp_start_east  = 0.0
        self.wp_start_speed = 0.0

        for i in range(1, 9):
            setattr(self, f"wp_{i}_north", 0.0)
            setattr(self, f"wp_{i}_east",  0.0)
            setattr(self, f"wp_{i}_speed", 0.0)

        self.wp_end_north = 0.0
        self.wp_end_east  = 0.0
        self.wp_end_speed = 0.0

        # Inputs
        self.north = 0.0
        self.east  = 0.0

        # Outputs
        self.prev_wp_north = 0.0
        self.prev_wp_east  = 0.0
        self.prev_wp_speed = 0.0

        self.next_wp_north = 0.0
        self.next_wp_east  = 0.0
        self.next_wp_speed = 0.0

        self.last_wp_active = False
        
        self.reach_wp_end   = False

        # Internal
        self._traj = []
        self._idx = 0
        self._traj_built = False

        # Registration
        self.register_variable(Real("ra", causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))
        self.register_variable(Integer("max_inter_wp", causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))

        self.register_variable(Real("north", causality=Fmi2Causality.input))
        self.register_variable(Real("east", causality=Fmi2Causality.input))

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

        self.register_variable(Boolean("last_wp_active", causality=Fmi2Causality.output))
        
        self.register_variable(Boolean("reach_wp_end", causality=Fmi2Causality.output))

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

        self._traj = traj
        self._idx = 0
        self._traj_built = True

        # Initialize outputs if we have at least 2 points
        if len(self._traj) >= 2:
            p = self._traj[0]
            q = self._traj[1]
            self.prev_wp_north, self.prev_wp_east, self.prev_wp_speed = p
            self.next_wp_north, self.next_wp_east, self.next_wp_speed = q
            self.last_wp_active = False
        else:
            # Not enough points
            self.prev_wp_north = self.prev_wp_east = self.prev_wp_speed = 0.0
            self.next_wp_north = self.next_wp_east = self.next_wp_speed = 0.0
            self.last_wp_active = False

    def _dist2_to_next(self) -> float:
        n2, e2, _ = self._traj[self._idx + 1]
        dn = n2 - float(self.north)
        de = e2 - float(self.east)
        return dn * dn + de * de

    def do_step(self, current_time: float, step_size: float) -> bool:
        try:
            # Build once (or rebuild if you want: detect changes; keeping it simple)
            if not self._traj_built:
                self._build_trajectory()

            # If not enough waypoints, nothing to do
            if len(self._traj) < 2:
                return True

            ra2 = float(self.ra) * float(self.ra)

            # If we're already on last segment, latch and keep outputs fixed
            # while keep checking if the ship reaches the end point
            if self._idx >= len(self._traj) - 2:
                self.last_wp_active = True
                self.reach_wp_end   = False
                
                # Check if the ship reaches the end point (three-quarter way inside the waypoint's RoA)
                if self._dist2_to_next() <= ra2 / 16:
                    self.last_wp_active = True
                    self.reach_wp_end   = True           
                return True

            # Switch ONLY when close to the NEXT waypoint
            if self._dist2_to_next() <= ra2:
                self._idx += 1

                # Update prev/next using new index
                p = self._traj[self._idx]
                q = self._traj[self._idx + 1] if (self._idx + 1) < len(self._traj) else self._traj[-1]

                self.prev_wp_north, self.prev_wp_east, self.prev_wp_speed = p
                self.next_wp_north, self.next_wp_east, self.next_wp_speed = q
            
        except Exception as e:
            # Keep host alive; do not crash co-sim
            print(f"[MissionManager] Exception t={current_time}, dt={step_size}: {type(e).__name__}: {e}")
            print(traceback.format_exc())

            # Freeze dynamics safely (keep last state/outputs)
            self.prev_wp_north = 0.0
            self.prev_wp_east  = 0.0
            self.prev_wp_speed = 0.0

            self.next_wp_north = 0.0
            self.next_wp_east  = 0.0
            self.next_wp_speed = 0.0
            
            self.last_wp_active = False
            
            self.reach_wp_end   = False
            
        return True

                
                