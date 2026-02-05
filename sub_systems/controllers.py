""" 
This module provides classes of controllers used to control the ship inside the simulator.
"""

import copy
import numpy as np
from typing import List, NamedTuple, Union

from simulator.ship_in_transit.sub_systems.LOS_guidance import NavigationSystem 

###################################################################################################################
#################################### CONFIGURATION FOR PID CONTROLLER #############################################
###################################################################################################################


class ThrottleControllerGains(NamedTuple):
    kp_ship_speed: float
    ki_ship_speed: float
    kp_shaft_speed: float
    ki_shaft_speed: float
    
    
class HeadingControllerGains(NamedTuple):
    kp: float
    kd: float
    ki: float

class LosParameters(NamedTuple):
    radius_of_acceptance: float
    lookahead_distance: float
    integral_gain: float
    integrator_windup_limit: float

def _wrap_pi(a):
    import math
    return (a + math.pi) % (2*math.pi) - math.pi

###################################################################################################################
###################################################################################################################


class PiController:
    def __init__(self, kp: float, ki: float, time_step: float, initial_integral_error=0):
        self.kp = kp
        self.ki = ki
        self.error_i = initial_integral_error
        self.time_step = time_step
        
        # Record initial parameters for reset purposes
        self.record_initial_parameters()

    def pi_ctrl(self, setpoint, measurement, *args):
        ''' Uses a proportional-integral control law to calculate a control
            output. The optional argument is an 2x1 array and will specify lower
            and upper limit for error integration [lower, upper]
        '''
        error = setpoint - measurement
        error_i = self.error_i + error * self.time_step
        if args:
            error_i = self.sat(error_i, args[0], args[1])
        self.error_i = error_i
        return error * self.kp + error_i * self.ki

    @staticmethod
    def sat(val, low, hi):
        ''' Saturate the input val such that it remains
        between "low" and "hi"
        '''
        return max(low, min(val, hi))
    
    def record_initial_parameters(self):
        '''
        Stores a deep copy of initial waypoint data for reset.
        '''
        self._initial_state = {
            'kp': copy.deepcopy(self.kp),  
            'ki': copy.deepcopy(self.ki),
            'error_i': copy.deepcopy(self.error_i),
            'time_step': copy.deepcopy(self.time_step)
        }
        
    def reset(self):
        '''
        Reset the PI controller for a new episode.
        '''
        self.kp = copy.deepcopy(self._initial_state['kp'])
        self.ki = copy.deepcopy(self._initial_state['ki'])
        self.error_i = copy.deepcopy(self._initial_state['error_i'])
        self.time_step = copy.deepcopy(self._initial_state['time_step'])

class PidController:
    def __init__(self, kp: float, kd: float, ki: float, time_step: float):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.error_i = 0
        self.prev_error = 0
        self.time_step = time_step
        
        # Record initial parameters for reset purposes
        self.record_initial_parameters()

    def pid_ctrl(self, setpoint, measurement, for_angle=False, *args):
        ''' Uses a proportional-derivative-integral control law to calculate
            a control output. The optional argument is a 2x1 array and will
            specify lower and upper [lower, upper] limit for error integration
        '''
        if for_angle is True:
            error = _wrap_pi(setpoint - measurement)
        else:
            error = setpoint - measurement
        d_error = (error - self.prev_error) / self.time_step
        error_i = self.error_i + error * self.time_step
        if args:
            error_i = self.sat(error_i, args[0], args[1])
        self.prev_error = error
        self.error_i = error_i
        return error * self.kp + d_error * self.kd + error_i * self.ki

    @staticmethod
    def sat(val, low, hi):
        ''' Saturate the input val such that it remains
        between "low" and "hi"
        '''
        return max(low, min(val, hi))

    def record_initial_parameters(self):
        '''
        Stores a deep copy of initial waypoint data for reset.
        '''
        self._initial_state = {
            'kp': copy.deepcopy(self.kp),  
            'ki': copy.deepcopy(self.ki),
            'kd': copy.deepcopy(self.kd),
            'error_i': copy.deepcopy(self.error_i),
            'prev_error': copy.deepcopy(self.prev_error),
            'time_step': copy.deepcopy(self.time_step)
        }

    def reset(self):
        '''
        Reset the PID controller for a new episode.
        '''
        self.kp = copy.deepcopy(self._initial_state['kp'])
        self.ki = copy.deepcopy(self._initial_state['ki'])
        self.kd = copy.deepcopy(self._initial_state['kd'])
        self.error_i = copy.deepcopy(self._initial_state['error_i'])
        self.prev_error = copy.deepcopy(self._initial_state['prev_error'])
        self.time_step = copy.deepcopy(self._initial_state['time_step'])
    
    
###################################################################################################################
################################## DESCENDANT CLASS FROM THE BASE CONTROLLER ######################################
###################################################################################################################


class EngineThrottleFromSpeedSetPoint:
    """
    Calculates throttle setpoint for power generation based on the ship´s speed, the propeller shaft speed
    and the desires ship speed.
    """

    def __init__(
            self,
            gains: ThrottleControllerGains,
            max_shaft_speed: float,
            time_step: float,
            initial_shaft_speed_integral_error: float
    ):
        # Initial internal attribute
        self.ship_speed_controller = PiController(
            kp=gains.kp_ship_speed, ki=gains.ki_ship_speed, time_step=time_step
        )
        self.shaft_speed_controller = PiController(
            kp=gains.kp_shaft_speed,
            ki=gains.ki_shaft_speed,
            time_step=time_step,
            initial_integral_error=initial_shaft_speed_integral_error
        )
        self.max_shaft_speed = max_shaft_speed
        
        # Record initial parameters for reset purposes
        self.record_initial_parameters()

    def throttle(self, speed_set_point, measured_speed, measured_shaft_speed):
        desired_shaft_speed = self.ship_speed_controller.pi_ctrl(setpoint=speed_set_point, measurement=measured_speed)
        desired_shaft_speed = self.ship_speed_controller.sat(val=desired_shaft_speed, low=0, hi=self.max_shaft_speed)
        throttle = self.shaft_speed_controller.pi_ctrl(setpoint=desired_shaft_speed, measurement=measured_shaft_speed)
        return self.shaft_speed_controller.sat(val=throttle, low=0, hi=1.1)
    
    def record_initial_parameters(self):
        '''
        Stores a deep copy of initial parameters for reset
        '''
        self._initial_state = {
            'max_shaft_speed': copy.deepcopy(self.max_shaft_speed)
        }
    
    def reset(self):
        ''' 
            Reset the internal attributes of the throttle controller
            its initial values
        '''
        self.max_shaft_speed = copy.deepcopy(self._initial_state['max_shaft_speed'])
        
        # Reset the PI controllers
        self.ship_speed_controller.reset()
        self.shaft_speed_controller.reset()


class ThrottleFromSpeedSetPointSimplifiedPropulsion:
    """
    Calculates throttle setpoint for power generation based on the ship´s speed, the propeller shaft speed
    and the desires ship speed.
    """

    def __init__(
            self,
            kp: float,
            ki: float,
            time_step: float,
    ):
        self.ship_speed_controller = PiController(
            kp=kp, ki=ki, time_step=time_step
        )

    def throttle(self, speed_set_point, measured_speed):
        throttle = self.ship_speed_controller.pi_ctrl(setpoint=speed_set_point, measurement=measured_speed)
        return self.ship_speed_controller.sat(val=throttle, low=0, hi=1.1)
    
    
class HeadingByReferenceController:
    def __init__(self, gains: HeadingControllerGains, time_step, max_rudder_angle, max_rudder_rate):
        self.gains = gains
        self.time_step = time_step
        self.ship_heading_controller = PidController(kp=self.gains.kp, 
                                                     kd=self.gains.kd, 
                                                     ki=self.gains.ki, 
                                                     time_step=self.time_step)
        # Limiter
        self.max_rudder_angle = max_rudder_angle
        self.max_rudder_rate  = float(max_rudder_rate)
        
        # Internal state: previously sent (post-limiter) command
        self.rudder_angle_cmd_prev = 0.0
        
        # Record initial parameters for reset purposes
        self.record_initial_parameters()
        
    def _apply_slew_limit(self, u_des, u_prev):
        """Limit |Δu| ≤ max_rudder_rate * dt, then hard-limit angle."""
        # Compute the maximum rudder angle displacement based on the maximum rudder angle rate
        max_delta = self.max_rudder_rate * self.time_step
        
        # Saturate the command input (desired - previous rudder angle command) w.r.t. the max_delta
        delta = self.ship_heading_controller.sat(u_des - u_prev, -max_delta, +max_delta)
        
        # Add the delta to the previous rudder angle command
        u_cmd = u_prev + delta
        
        # Final angle saturation
        return self.ship_heading_controller.sat(u_cmd, -self.max_rudder_angle, +self.max_rudder_angle)    
    
    def rudder_angle_from_heading_setpoint(self, heading_ref: float, measured_heading: float):
        '''
            PID -> desired rudder -> rate limit -> angle limit.
            IMPORTANT: keep units consistent (all deg or all rad).
            
            This method finds a suitable rudder angle for the ship to
            sail with the heading specified by "heading_ref" by using
            PID-controller. The rudder angle is saturated according to
            |self.rudder_ang_max|. The method should be called from within
            simulation loop if the user want the ship to follow a specified
            heading reference signal.
        '''
        
        rudder_angle_des = -self.ship_heading_controller.pid_ctrl(setpoint=heading_ref, measurement=measured_heading,
                                                                  for_angle=True)
        
        # Apply slew limiter around the *previous sent* command
        rudder_angle_cmd = self._apply_slew_limit(rudder_angle_des, self.rudder_angle_cmd_prev)
        
        # store and return
        self.rudder_angle_cmd_prev = rudder_angle_cmd
        
        return rudder_angle_cmd
    
    def record_initial_parameters(self):
        '''
        Stores a deep copy of heading controller for reset.
        '''
        self._initial_state = {
            'max_rudder_angle': copy.deepcopy(self.max_rudder_angle),
            'max_rudder_rate': copy.deepcopy(self.max_rudder_rate),  
            'rudder_angle_cmd_prev': 0.0,
        }
    
    def reset(self):
        ''' 
            Reset the internal attributes of the heading controller
            its initial values
        '''
        self.max_rudder_angle = copy.deepcopy(self._initial_state['max_rudder_angle'])
        self.max_rudder_rate  = copy.deepcopy(self._initial_state['max_rudder_rate'])
        self.rudder_angle_cmd_prev = copy.deepcopy(self._initial_state['rudder_angle_cmd_prev'])
        
        # Reset the heading controller
        self.ship_heading_controller.reset()
        

class HeadingByRouteController:
    def __init__(
            self, route_name,
            heading_controller_gains: HeadingControllerGains,
            los_parameters: LosParameters,
            time_step: float,
            max_rudder_angle: float,
    ):
        self.heading_controller = HeadingByReferenceController(
            gains=heading_controller_gains, time_step=time_step, max_rudder_angle=max_rudder_angle
        )
        self.navigate = NavigationSystem(
            route=route_name,
            radius_of_acceptance=los_parameters.radius_of_acceptance,
            lookahead_distance=los_parameters.lookahead_distance,
            integral_gain=los_parameters.integral_gain,
            integrator_windup_limit=los_parameters.integrator_windup_limit
        )
        ## Initial internal attributes
        self.next_wpt = 1
        self.prev_wpt = 0
        
        self.heading_ref = 0
        self.heading_mea = 0
        
        # Record initial parameters for reset purposes
        self.record_initial_parameters()
        
    def rudder_angle_from_route(self, north_position, east_position, heading):
        ''' This method finds a suitable rudder angle for the ship to follow
            a predefined route specified in the "navigate"-instantiation of the
            "NavigationSystem"-class.
        '''
        self.next_wpt, self.prev_wpt = self.navigate.next_wpt(self.next_wpt, north_position, east_position)
        self.heading_ref = self.navigate.los_guidance(self.next_wpt, north_position, east_position)
        self.heading_mea = heading
        return self.heading_controller.rudder_angle_from_heading_setpoint(heading_ref=self.heading_ref, measured_heading=heading)
    
    ## ADDITIONAL ##
    def record_initial_parameters(self):
        '''
        Stores a deep copy of heading controller for reset.
        '''
        self._initial_state = {
            'next_wpt': 1,
            'prev_wpt': 0,
            'heading_ref': 0,
            'heading_mea': 0
        }
    
    def reset(self):
        '''
        Reset the heading controller for a new episode.
        '''
        # Internal attributes reset
        self.next_wpt = self._initial_state['next_wpt']
        self.prev_wpt = self._initial_state['prev_wpt']
        
        self.heading_ref = self._initial_state['heading_ref']
        self.heading_mea = self._initial_state['heading_mea']
        
        # Heading controller reset
        self.heading_controller.reset()
        
        # Navigation system reset
        self.navigate.reset()
    

class HeadingBySampledRouteController:
    def __init__(
            self, route_name,
            heading_controller_gains: HeadingControllerGains,
            los_parameters: LosParameters,
            time_step: float,
            max_rudder_angle: float,
            max_rudder_rate:float
    ):
        
        self.heading_controller = HeadingByReferenceController(
            gains=heading_controller_gains, time_step=time_step, max_rudder_angle=max_rudder_angle, max_rudder_rate=max_rudder_rate
        )
        self.navigate = NavigationSystem(
            route=route_name,
            radius_of_acceptance=los_parameters.radius_of_acceptance,
            lookahead_distance=los_parameters.lookahead_distance,
            integral_gain=los_parameters.integral_gain,
            integrator_windup_limit=los_parameters.integrator_windup_limit,
        )
        
        ## Initial internal attributes
        self.next_wpt = 1
        self.prev_wpt = 0
        
        self.heading_ref = 0
        self.heading_mea = 0
        
        # Record initial parameters for reset purposes
        self.record_initial_parameters()
        
    def update_route(self, intermediate_waypoints):
        # Get the route shifts and update the new route
        IW_n, IW_e = intermediate_waypoints
        
        self.navigate.north.insert(-1, IW_n)
        self.navigate.east.insert(-1, IW_e)
        

    def rudder_angle_from_sampled_route(self, north_position, east_position, heading, desired_heading_offset=0.0):
        ''' This method finds a suitable rudder angle for the ship to follow
            a predefined route specified in the "navigate"-instantiation of the
            "NavigationSystem"-class.
        '''
        self.next_wpt, self.prev_wpt = self.navigate.next_wpt(self.next_wpt, north_position, east_position)
        self.heading_ref = self.navigate.los_guidance(self.next_wpt, north_position, east_position)
        self.heading_mea = heading
        return self.heading_controller.rudder_angle_from_heading_setpoint(heading_ref=self.heading_ref + desired_heading_offset, measured_heading=heading)
    
    ## ADDITIONAL ##
    def get_heading_error(self):
        return np.abs(self.heading_mea - self.heading_ref)
    
    def get_cross_track_error(self):
        return self.navigate.e_ct
    
    def record_initial_parameters(self):
        '''
        Stores a deep copy of heading controller for reset.
        '''
        self._initial_state = {
            'next_wpt': 1,
            'prev_wpt': 0,
            'heading_ref': 0,
            'heading_mea': 0
        }
    
    def reset(self, route=None):
        '''
        Reset the heading controller for a new episode.
        '''
        # Internal attributes reset
        self.next_wpt = self._initial_state['next_wpt']
        self.prev_wpt = self._initial_state['prev_wpt']
        
        self.heading_ref = self._initial_state['heading_ref']
        self.heading_mea = self._initial_state['heading_mea']
        
        # Heading controller reset
        self.heading_controller.reset()
        
        # Navigation system reset
        self.navigate.reset(route)
        
        