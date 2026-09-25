"""
Shaft Speed Controller Python FMU implementation.
This FMU manages a shaft speed controller based on PI Controller.

Authors : Andreas R.G. Sitorus
Date    : Januari 2026
"""

from pythonfmu import Fmi2Causality, Fmi2Slave, Fmi2Variability, Real, Integer, Boolean, String
import numpy as np
import traceback

class ShaftSpeedController(Fmi2Slave):
    
    author = "Andreas R.G. Sitorus"
    description = "PI-Based Shaft Speed Controller Python FMU Implementation"
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        
        ## PI Parameters
        self.kp                                             = 0.0
        self.ki                                             = 0.0
        
        self.rated_speed_main_engine_rpm                    = 0.0
        self.gear_ratio_between_main_engine_and_propeller   = 0.0
        self.idle_rpm_fraction                              = 0.0
        
        # Internal Variables
        self.error_i                                        = 0.0
        self.prev_error                                     = 0.0
        
        ## Input
        self.desired_ship_speed                             = 0.0
        self.measured_ship_speed                            = 0.0
        
        ## Output
        self.shaft_speed_cmd_rpm                            = 0.0
        
        self.rpm_cmd_pi                                     = 0.0
        self.rpm_cmd_ff                                     = 0.0
        self.error                                          = 0.0
        
        self.des_speed                                      = 0.0
        self.mea_speed                                      = 0.0
        
        ## Registration
        # PI Parameters
        self.register_variable(Real("kp", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
        self.register_variable(Real("ki", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
        
        self.register_variable(Real("rated_speed_main_engine_rpm", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        self.register_variable(Real("gear_ratio_between_main_engine_and_propeller", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        self.register_variable(Real("idle_rpm_fraction", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        
        # Input
        self.register_variable(Real("desired_ship_speed", causality=Fmi2Causality.input))
        self.register_variable(Real("measured_ship_speed", causality=Fmi2Causality.input))

        # Output
        self.register_variable(Real("shaft_speed_cmd_rpm", causality=Fmi2Causality.output))
        # - debug 
        self.register_variable(Real("rpm_cmd_pi", causality=Fmi2Causality.output))
        self.register_variable(Real("rpm_cmd_ff", causality=Fmi2Causality.output))
        self.register_variable(Real("error_i", causality=Fmi2Causality.output))
        self.register_variable(Real("error", causality=Fmi2Causality.output))
        self.register_variable(Real("des_speed", causality=Fmi2Causality.output))
        self.register_variable(Real("mea_speed", causality=Fmi2Causality.output))
        

    @staticmethod
    def sat(val, low, hi):
        ''' Saturate the input val such that it remains
        between "low" and "hi"
        '''
        return max(low, min(val, hi))
    
    
    def map_speed_to_rpm(self, desired_ship_speed, shaft_speed_floor_rpm):
        # Testing data
        rpm_data   = np.array([ 66, 100, 200, 300, 400, 450, 500, 600, 660], dtype=float)
        v_data     = np.array([0.18,0.407,1.533,3.168,5.14,6.2, 6.72,6.72,6.72], dtype=float)
        
        # Remove saturated data
        mask    = v_data< 6.72
        rpm_lin = rpm_data[mask]
        v_lin   = v_data[mask]
        
        # Get the a by finding which segment desired_ship_speed falls into and return that segment slope
        vq  = float(np.clip(desired_ship_speed, v_lin.min(), v_lin.max()))  # Get the v_query
        idx = np.searchsorted(v_lin, vq) - 1                                # Get the idx in which the v_query fell into (lower bound)
        idx = int(np.clip(idx, 0, len(v_lin)-2))                            # Make sure that the idx do not preceed 0 and exceed the max idx of the data
        dv  = v_lin[idx+1] - v_lin[idx]                                     # Get the change of ship velocity
        dr  = rpm_lin[idx+1] - rpm_lin[idx]                                 # Get the change of shaft speed
        a   = float(dr / dv)                                                # Get the rate of rpm change w.r.t. change of ship speed
            
        # Get the feed forward rpm (from linear mapping)
        # Formula: a [rpm/(m/s)] * desired_ship_speed [m/s] + shaft_speed_floor_rpm [rpm]
        # rpm_cmd_ff = a * desired_ship_speed + shaft_speed_floor_rpm
        rpm_cmd_ff = a + shaft_speed_floor_rpm
        return rpm_cmd_ff
        
    
    
    def do_step(self, current_time: float, step_size: float) -> bool:
        try:
            # Max shaft speed
            shaft_speed_max_rpm     = 1.1 * self.rated_speed_main_engine_rpm * self.gear_ratio_between_main_engine_and_propeller
            shaft_speed_floor_rpm   = self.idle_rpm_fraction * shaft_speed_max_rpm     # Floor shaft speed ~ 10 % max rpm
            
            # Get PI rpm command            
            error       = self.desired_ship_speed - self.measured_ship_speed
            rpm_cmd_p   = self.kp * error
            rpm_cmd_i   = self.ki * self.error_i
            rpm_cmd_pi  = rpm_cmd_p + rpm_cmd_i
            
            # Get feedforward rpm command
            rpm_cmd_ff = self.map_speed_to_rpm(self.desired_ship_speed, shaft_speed_floor_rpm)
            
            # Get the unsaturated rpm command
            rpm_cmd_unsat = rpm_cmd_ff + rpm_cmd_pi
            
            # Saturate the rpm command
            rpm_cmd_sat = self.sat(rpm_cmd_unsat, 0.0, shaft_speed_max_rpm)
            
            # Anti-windup
            saturated_high = (rpm_cmd_sat >= shaft_speed_max_rpm - 1e-9)
            saturated_low  = (rpm_cmd_sat <= shaft_speed_floor_rpm + 1e-9)
            
            # If high-saturated and error wants to increase command further -> don't integrate.
            # If low-saturated and error wants to decrease command further -> don't integrate.
            pushing_further_high = saturated_high and (error > 0.0)
            pushing_further_low  = saturated_low  and (error < 0.0)
            
            # If command IS saturated, do not integrate the error
            if not (pushing_further_high or pushing_further_low):
                self.error_i += error * step_size
                
            # - debug
            self.rpm_cmd_pi = rpm_cmd_pi
            self.rpm_cmd_ff = rpm_cmd_ff
            self.error      = error
            self.des_speed  = self.desired_ship_speed
            self.mea_speed  = self.measured_ship_speed
                
            # Output
            self.shaft_speed_cmd_rpm = rpm_cmd_sat
        
        except Exception as e:
            # IMPORTANT: do not crash host
            print(f"[ShaftSpeedController] ERROR t={current_time} dt={step_size}: {type(e).__name__}: {e}")
            print(traceback.format_exc())
            
            # Freeze dynamics safely (keep last state/outputs)
            self.shaft_speed_cmd_rpm    = 0.0
        
        return True