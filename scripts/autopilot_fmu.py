"""
Autopilot Python FMU implementation.
This FMU manages a heading controller (autopilot) based on PID Controller with Line-of-Sight Guidance.

Authors : Andreas R.G. Sitorus
Date    : Januari 2026
"""

from pythonfmu import Fmi2Causality, Fmi2Slave, Fmi2Variability, Real, Integer, Boolean, String
import math

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
        
        self.next_wp_north                  = 0.0
        self.next_wp_east                   = 0.0
        
        self.prev_wp_north                  = 0.0
        self.prev_wp_east                   = 0.0
        
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
        return (a + math.pi) % (2*math.pi) - math.pi

    
    def sat(self, val, low, hi):
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
        error = self._wrap_to_pi(setpoint - measurement)   # Compute angle error, wrap to pi
        d_error = (error - self.prev_error) / step_size if step_size > 0 else 0.0
        error_i = self.error_i + error * step_size
        
        if lim_low is not None and lim_hi is not None: 
            error_i = self.sat(error_i, lim_low, lim_hi)
            
        self.prev_error = error
        self.error_i = error_i
        return error * self.kp + d_error * self.kd + error_i * self.ki
    

    def apply_slew_limit(self, u_des_rad: float, u_prev_rad: float, step_size: float) -> float:
        """Limit |Δu| ≤ max_rudder_rate * dt, then hard-limit angle."""
        max_rate_rad = math.radians(self.max_rudder_rate_deg_per_sec)
        max_angle_rad = math.radians(self.max_rudder_angle_deg)
        
        # Compute the maximum rudder angle displacement based on the maximum rudder angle rate
        max_delta = max_rate_rad * step_size
        
        # Saturate the command input (desired - previous rudder angle command) w.r.t. the max_delta
        delta = self.sat(u_des_rad - u_prev_rad, -max_delta, +max_delta)
        
        # Add the delta to the previous rudder angle command
        u_cmd = u_prev_rad + delta
        
        # Final angle saturation
        return self.sat(u_cmd, -max_angle_rad, +max_angle_rad)  
    
    
    def los_guidance(self, north, east, next_wp_north, next_wp_east, prev_wp_north, prev_wp_east):
        dx = next_wp_east - prev_wp_east
        dy = next_wp_north - prev_wp_north
        
        alpha_k = math.atan2(dy, dx)
        
        # Cross-Track Error
        e_ct = -(north - next_wp_north) * math.sin(alpha_k) + (east - next_wp_east) * math.cos(alpha_k)
        
        # Keep inside circle
        if e_ct ** 2 >= self.r ** 2 and self.r > 0:
            e_ct = math.copysign(0.99*self.r, e_ct)
        
        inside = self.r ** 2 - e_ct ** 2
        inside = max(0.0, inside)
        delta = max(1e-6, math.sqrt(inside))
        
        # Integrator anti-windup
        candidate = self.e_ct_int + e_ct / delta
        if abs(candidate) <= self.integrator_limit:
            self.e_ct_int = candidate
            
        chi_r = math.atan2(-e_ct, delta - self.e_ct_int * self.ki_ct)
        
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
        rudder_angle_rad = self.apply_slew_limit(rudder_angle_des_rad, self.prev_rudder_angle_rad, step_size=step_size)
        
        # Store current to previous rudder angle command
        self.prev_rudder_angle_rad = rudder_angle_rad
        
        # Outputs
        self.yaw_angle_ref_rad  = yaw_angle_ref_rad
        self.e_ct               = e_ct
        self.rudder_angle_deg   = math.degrees(rudder_angle_rad)
        
        return True
        
        