"""
Speed Controller Python FMU implementation.
This FMU manages a shaft speed controller based on PID Controller.

Authors : Andreas R.G. Sitorus
Date    : Januari 2026
"""

from pythonfmu import Fmi2Causality, Fmi2Slave, Fmi2Variability, Real, Integer, Boolean, String
import numpy as np
import traceback

class SpeedController(Fmi2Slave):
    
    author = "Andreas R.G. Sitorus"
    description = "PI-Based Speed Controller Python FMU Implementation"
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        
        ## PI Parameters
        self.kp                                             = 0.0
        self.ki                                             = 0.0
        self.kd                                             = 0.0
        
        # Internal Variables
        self.error_i                                        = 0.0
        self.prev_error                                     = 0.0
        
        ## Input
        self.desired_ship_speed                             = 0.0
        self.measured_ship_speed                            = 0.0
        
        ## Output
        self.thrust_force                                   = 0.0
        
        ## Registration
        # PI Parameters
        self.register_variable(Real("kp", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
        self.register_variable(Real("ki", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
        self.register_variable(Real("kd", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
        
        # Input
        self.register_variable(Real("desired_ship_speed", causality=Fmi2Causality.input))
        self.register_variable(Real("measured_ship_speed", causality=Fmi2Causality.input))

        # Output
        self.register_variable(Real("thrust_force", causality=Fmi2Causality.output))

    @staticmethod
    def sat(val, low, hi):
        ''' Saturate the input val such that it remains
        between "low" and "hi"
        '''
        return max(low, min(val, hi))
    
    
    def pid_ctrl(self, setpoint: float, measurement: float, step_size: float, lim_low=None, lim_hi=None) -> float:
        ''' 
            Uses a proportional-derivative-integral control law to calculate
            a control output. The optional argument is a 2x1 array and will
            specify lower and upper [lower, upper] limit for error integration
        '''
        error = setpoint - measurement   # Compute angle error, wrap to pi
        d_error = (error - self.prev_error) / step_size if step_size > 0 else 0.0
        error_i = self.error_i + error * step_size
        
        if lim_low is not None and lim_hi is not None: 
            error_i = self.sat(error_i, lim_low, lim_hi)
            
        self.prev_error = error
        self.error_i = error_i
        return (error * self.kp + d_error * self.kd + error_i * self.ki)
    
    
    def do_step(self, current_time: float, step_size: float) -> bool:
        try:
            # Get thrust force
            self.thrust_force = self.pid_ctrl(setpoint=self.desired_ship_speed,
                                              measurement=self.measured_ship_speed,
                                              step_size=step_size)
        
        except Exception as e:
            # IMPORTANT: do not crash host
            print(f"[SpeedController] ERROR t={current_time} dt={step_size}: {type(e).__name__}: {e}")
            print(traceback.format_exc())
            
            # Freeze dynamics safely (keep last state/outputs)
            self.thrust_force = 0.0
        
        return True