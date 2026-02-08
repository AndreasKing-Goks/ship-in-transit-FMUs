"""
Shaft Speed Controller Python FMU implementation.
This FMU manages a shaft speed controller based on PI Controller.

Authors : Andreas R.G. Sitorus
Date    : Januari 2026
"""

from pythonfmu import Fmi2Causality, Fmi2Slave, Fmi2Variability, Real, Integer, Boolean, String
import traceback

class ShaftSpeedController(Fmi2Slave):
    
    author = "Andreas R.G. Sitorus"
    description = "PI-Based Shaft Speed Controller Python FMU Implementation"
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        
        ## PI Parameters
        self.kp                     = 0.0
        self.ki                     = 0.0
        
        # Internal Variables
        self.error_i                = 0.0
        self.prev_error             = 0.0
        
        ## Input
        self.desired_ship_speed     = 0.0
        self.measured_ship_speed    = 0.0
        
        ## Output
        self.shaft_speed_cmd_rpm    = 0.0
        
        ## Registration
        # PI Parameters
        self.register_variable(Real("kp", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
        self.register_variable(Real("ki", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
        
        # Input
        self.register_variable(Real("desired_ship_speed", causality=Fmi2Causality.input))
        self.register_variable(Real("measured_ship_speed", causality=Fmi2Causality.input))

        # Output
        self.register_variable(Real("shaft_speed_cmd_rpm", causality=Fmi2Causality.output))
    
    
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
            desired_shaft_speed_rpm = self.pi_ctrl(setpoint=self.desired_ship_speed, 
                                                measurement=self.measured_ship_speed,
                                                step_size=step_size)       
            self.shaft_speed_cmd_rpm = desired_shaft_speed_rpm
        except Exception as e:
            # IMPORTANT: do not crash host
            print(f"[ShaftSpeedController] ERROR t={current_time} dt={step_size}: {type(e).__name__}: {e}")
            print(traceback.format_exc())
            
            # Freeze dynamics safely (keep last state/outputs)
            self.shaft_speed_cmd_rpm    = 0.0
        
        return True