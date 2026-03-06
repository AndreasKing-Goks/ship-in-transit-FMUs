"""
Throttle Controller Python FMU implementation.
This FMU manages a throttle controller based on PI Controller.

Authors : Andreas R.G. Sitorus
Date    : Januari 2026
"""

from pythonfmu import Fmi2Causality, Fmi2Slave, Fmi2Variability, Real, Integer, Boolean, String
import traceback

class ThrottleController(Fmi2Slave):
    
    author = "Andreas R.G. Sitorus"
    description = "PI-Based Throttle Controller Python FMU Implementation"
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        
        ## PI Parameters
        self.kp                         = 0.0
        self.ki                         = 0.0
        
        # Internal Variables
        self.error_i                    = 0.0
        self.prev_error                 = 0.0
        
        ## Input
        self.desired_shaft_speed_rpm    = 0.0
        self.measured_shaft_speed_rpm   = 0.0 
        
        ## Output
        self.throttle_cmd               = 0.0
        
        ## Registration
        # PI Parameters
        self.register_variable(Real("kp", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        self.register_variable(Real("ki", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        
        # Input
        self.register_variable(Real("desired_shaft_speed_rpm", causality=Fmi2Causality.input))
        self.register_variable(Real("measured_shaft_speed_rpm", causality=Fmi2Causality.input))

        # Output
        self.register_variable(Real("throttle_cmd", causality=Fmi2Causality.output))
        
    
    @staticmethod
    def sat(val, low, hi):
        ''' Saturate the input val such that it remains
        between "low" and "hi"
        '''
        return max(low, min(val, hi))
    
    
    def do_step(self, current_time: float, step_size: float) -> bool:
        try:
            # error in RPM domain (inner loop)
            error = self.desired_shaft_speed_rpm - self.measured_shaft_speed_rpm

            # PI unsaturated
            u_unsat = self.kp * error + self.ki * self.error_i

            # Saturate throttle (recommended)
            u_sat = self.sat(u_unsat, 0.0, 1.0)

            # Anti-windup: conditional integration
            sat_high = (u_sat >= 1.0 - 1e-9)
            sat_low  = (u_sat <= 0.0 + 1e-9)

            pushing_further_high = sat_high and (error > 0.0)  # wants more throttle but already max
            pushing_further_low  = sat_low  and (error < 0.0)  # wants less throttle but already min

            if not (pushing_further_high or pushing_further_low):
                self.error_i += error * step_size

            self.throttle_cmd = u_sat
            
        except Exception as e:
            # IMPORTANT: do not crash host
            print(f"[ThrottleController] ERROR t={current_time} dt={step_size}: {type(e).__name__}: {e}")
            print(traceback.format_exc())
            
            # Freeze dynamics safely (keep last state/outputs)
            self.throttle_cmd = 0.0
            
        return True