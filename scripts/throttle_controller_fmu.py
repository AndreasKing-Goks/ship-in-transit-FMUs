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
    
    
    def pi_ctrl(self, setpoint, measurement, step_size, *args):
        ''' 
            Uses a proportional-integral control law to calculate a control
            output. The optional argument is an 2x1 array and will specify lower
            and upper limit for error integration [lower, upper]
        '''
        error = setpoint - measurement
        error_i = self.error_i + error * step_size
        if args:
            error_i = self.sat(error_i, args[0], args[1])
        self.error_i = error_i
        return error * self.kp + error_i * self.ki
    
    
    def do_step(self, current_time: float, step_size: float) -> bool:
        try:
            throttle = self.pi_ctrl(setpoint=self.desired_shaft_speed_rpm, 
                                    measurement=self.measured_shaft_speed_rpm,
                                    step_size=step_size)
            
            self.throttle_cmd = self.sat(val=throttle, low=0, hi=1.1)
            
        except Exception as e:
            # IMPORTANT: do not crash host
            print(f"[ThrottleController] ERROR t={current_time} dt={step_size}: {type(e).__name__}: {e}")
            print(traceback.format_exc())
            
            # Freeze dynamics safely (keep last state/outputs)
            self.throttle_cmd               = 0.0
        return True