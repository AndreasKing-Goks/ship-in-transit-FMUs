"""
Autopilot Python FMU implementation.
This FMU manages a heading controller (autopilot) based on PID Controller with Line-of-Sight Guidance.

Authors : Andreas R.G. Sitorus
Date    : Januari 2026
"""

from pythonfmu import Fmi2Causality, Fmi2Slave, Fmi2Variability, Real, Integer, Boolean, String
import numpy as np

class Autopilot(Fmi2Slave):
    
    author = "Andreas R.G. Sitorus"
    description = "PID-Based Heading Controller with Line-of-Sight Guidance Algorithm"
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        
        ## LOS Guidance Parameters
        self.ra                             = 0.0       # Radius of Acceptance
        self.r                              = 0.0       # Lookahead Distance
        self.ki_ct                          = 0.0       # Integral Gain
        self.integrator_limit               = 0.0       # Integrator Windup Limit
        
        ## Autopilot Parameters
        self.kp                             = 0.0
        self.ki                             = 0.0
        self.kd                             = 0.0
        
        self.max_rudder_rate_deg_per_sec    = 0.0
        self.max_rudder_angle_deg           = 0.0
        
        # Internal Variables
        self.e_ct_int                       = 0.0       # Cross Track Error Integral Value
        self.error_i                        = 0.0
        self.prev_error                     = 0.0
        self.prev_rudder_angle_rad          = 0.0       # Previous Rudder Angle Command, initiated at zero degree
        
        ## Input
        self.north                          = 0.0
        self.east                           = 0.0
        self.yaw_angle_rad                  = 0.0
        
        self.next_wp_north                  = np.nan
        self.next_wp_east                   = np.nan
        
        self.prev_wp_north                  = np.nan
        self.prev_wp_east                   = np.nan
        
        ## Output
        self.yaw_angle_ref_rad              = 0.0       # Heading Reference
        self.rudder_angle_deg               = 0.0       # Rudder Angle
        self.e_ct                           = 0.0       # Cross-Track Error
        
        ## Registration
        # LOS Guidance Parameters
        self.register_variable(Real("ra", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
        self.register_variable(Real("r", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
        self.register_variable(Real("ki_ct", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        self.register_variable(Real("integrator_limit", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        
        # PID Parameters
        self.register_variable(Real("kp", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
        self.register_variable(Real("ki", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
        self.register_variable(Real("kd", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
        self.register_variable(Real("max_rudder_rate_deg_per_sec", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        self.register_variable(Real("max_rudder_angle_deg", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        
        # Input
        self.register_variable(Real("north", causality=Fmi2Causality.input))
        self.register_variable(Real("east", causality=Fmi2Causality.input))
        self.register_variable(Real("yaw_angle_rad", causality=Fmi2Causality.input))
        
        self.register_variable(Real("next_wp_north", causality=Fmi2Causality.input))
        self.register_variable(Real("next_wp_east", causality=Fmi2Causality.input))
        
        self.register_variable(Real("prev_wp_north", causality=Fmi2Causality.input))
        self.register_variable(Real("prev_wp_east", causality=Fmi2Causality.input))
        
        # Output
        self.register_variable(Real("yaw_angle_ref_rad", causality=Fmi2Causality.output))
        self.register_variable(Real("rudder_angle_deg", causality=Fmi2Causality.output))
        self.register_variable(Real("e_ct", causality=Fmi2Causality.output))
        
    
    def _wrap_to_pi(self, a):
        return (a + np.pi) % (2*np.pi) - np.pi
    
    
    def sat(val, low, hi):
        ''' Saturate the input val such that it remains
        between "low" and "hi"
        '''
        return max(low, min(val, hi))

    
    def pid_ctrl(self, setpoint, measurement, step_size, *args):
        ''' 
            Uses a proportional-derivative-integral control law to calculate
            a control output. The optional argument is a 2x1 array and will
            specify lower and upper [lower, upper] limit for error integration
        '''
        error = self._wrap_pi(setpoint - measurement)   # Compute angle error, wrap to pi
        d_error = (error - self.prev_error) / step_size
        error_i = self.error_i + error * step_size
        if args:
            error_i = self.sat(error_i, args[0], args[1])
        self.prev_error = error
        self.error_i = error_i
        return error * self.kp + d_error * self.kd + error_i * self.ki
    

    def _apply_slew_limit(self, u_des, u_prev, step_size):
        """Limit |Δu| ≤ max_rudder_rate * dt, then hard-limit angle."""
        # Compute the maximum rudder angle displacement based on the maximum rudder angle rate
        max_delta = self.max_rudder_rate * step_size
        
        # Saturate the command input (desired - previous rudder angle command) w.r.t. the max_delta
        delta = self.sat(u_des - u_prev, -max_delta, +max_delta)
        
        # Add the delta to the previous rudder angle command
        u_cmd = u_prev + delta
        
        # Final angle saturation
        return self.sat(u_cmd, -self.max_rudder_angle, +self.max_rudder_angle)  
    
    
    def los_guidance(self, north, east, next_wp_north, next_wp_east, prev_wp_north, prev_wp_east):
        dx = next_wp_east - prev_wp_east
        dy = next_wp_north - prev_wp_north
        
        alpha_k = np.atan2(dx, dy)
        
        e_ct = -(north - next_wp_north) * np.sin(alpha_k) + (east - next_wp_east) * np.cos(alpha_k)
        
        if e_ct ** 2 >= self.r ** 2:
            e_ct = np.copysign(0.99*self.r, e_ct)
        
        delta = max(1e-6, np.sqrt(self.r ** 2 - e_ct ** 2))
        
        if abs(self.e_ct_int + e_ct / delta) <= self.integrator_limit:
            self.e_ct_int += e_ct / delta
            
        chi_r = np.atan2(-e_ct, delta - self.e_ct_int*self.ki_ct)
        
        yaw_angle_ref_rad = self._wrap_to_pi(alpha_k + chi_r)
        
        return yaw_angle_ref_rad, e_ct
    
    
    def do_step(self, current_time: float, step_size: float) -> bool:
        # Get the heading reference and the measured heading using input
        yaw_angle_ref_rad, e_ct = self.los_guidance(self.north, self.east, 
                                              self.next_wp_north, self.next_wp_east, 
                                              self.prev_wp_north, self.prev_wp_east)
        
        # Compute the control signal
        rudder_angle_des_rad = self.pid_ctrl(setpoint=yaw_angle_ref_rad, measurement=self.yaw_angle_rad, step_size=step_size) # self.heading_mea is input
        
        # Apply slew limiter around the previous rudder command
        rudder_angle_rad = self._apply_slew_limit(rudder_angle_des_rad, self.prev_rudder_angle_rad, step_size=step_size)
        
        # Store current to previous rudder angle command
        self.prev_rudder_angle_rad = rudder_angle_rad
        
        # Outputs
        self.yaw_angle_ref_rad  = yaw_angle_ref_rad
        self.e_ct               = e_ct
        self.rudder_angle_deg   = np.rad2deg(rudder_angle_rad)
        
        return True
        
        