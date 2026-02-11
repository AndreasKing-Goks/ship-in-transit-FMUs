"""
Ship Model Python FMU implementation.
This FMU manages the kinetic and kinematic models of the ship.

Authors : Andreas R.G. Sitorus
Date    : Januari 2026
"""

from pythonfmu import Fmi2Causality, Fmi2Slave, Fmi2Variability, Real, Integer, Boolean, String
import numpy as np
import traceback

class ShipModel(Fmi2Slave):
    
    author = "Andreas R.G. Sitorus"
    description = "Ship Model Python FMU Implementation"
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        
        # Precomputed Flag
        self._precomputed                                   = False
        
        ## Parameters
        # Ship configuration
        self.dead_weight_tonnage                            = 0.0
        self.coefficient_of_deadweight_to_displacement      = 0.0 
        self.bunkers                                        = 0.0
        self.ballast                                        = 0.0
        self.length_of_ship                                 = 0.0
        self.width_of_ship                                  = 0.0
        self.added_mass_coefficient_in_surge                = 0.0
        self.added_mass_coefficient_in_sway                 = 0.0
        self.added_mass_coefficient_in_yaw                  = 0.0
        self.mass_over_linear_friction_coefficient_in_surge = 0.0
        self.mass_over_linear_friction_coefficient_in_sway  = 0.0
        self.mass_over_linear_friction_coefficient_in_yaw   = 0.0
        self.nonlinear_friction_coefficient_in_surge        = 0.0
        self.nonlinear_friction_coefficient_in_sway         = 0.0
        self.nonlinear_friction_coefficient_in_yaw          = 0.0
        
        # Environment
        self.rho_seawater                                   = 1025.0
        self.rho_air                                        = 1.2
        self.g                                              = 9.81
        self.front_above_water_height                       = 0.0
        self.side_above_water_height                        = 0.0
        self.cx                                             = 0.0
        self.cy                                             = 0.0
        self.cn                                             = 0.0
        
        # Initial states
        self.initial_north_position_m                       = 0.0
        self.initial_east_position_m                        = 0.0
        self.initial_yaw_angle_rad                          = 0.0
        self.initial_forward_speed_m_per_s                  = 0.0
        self.initial_sideways_speed_m_per_s                 = 0.0
        self.initial_yaw_rate_rad_per_s                     = 0.0
        
        # Input variables
        self.thrust_force                                   = 0.0
        self.rudder_force_v                                 = 0.0
        self.rudder_force_r                                 = 0.0
        self.wind_speed                                     = 0.0
        self.wind_dir_rad                                   = 0.0
        self.current_speed                                  = 0.0
        self.current_dir_rad                                = 0.0
        
        # Output variables
        self.north                                          = 0.0
        self.east                                           = 0.0
        self.yaw_angle_rad                                  = 0.0
        self.forward_speed                                  = 0.0
        self.sideways_speed                                 = 0.0
        self.yaw_rate                                       = 0.0
        
        self.measured_ship_speed                            = 0.0
        
        self.d_north                                        = 0.0
        self.d_east                                         = 0.0
        self.d_yaw_angle_rad                                = 0.0
        self.d_forward_speed                                = 0.0
        self.d_sideways_speed                               = 0.0
        self.d_yaw_rate                                     = 0.0
        
        ## Registration
        # =========================
        # Ship configuration (parameters, fixed)
        # =========================
        self.register_variable(Real("dead_weight_tonnage", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("coefficient_of_deadweight_to_displacement", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("bunkers", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("ballast", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("length_of_ship", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("width_of_ship", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("added_mass_coefficient_in_surge", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("added_mass_coefficient_in_sway", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("added_mass_coefficient_in_yaw", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("mass_over_linear_friction_coefficient_in_surge", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("mass_over_linear_friction_coefficient_in_sway", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("mass_over_linear_friction_coefficient_in_yaw", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("nonlinear_friction_coefficient_in_surge", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("nonlinear_friction_coefficient_in_sway", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("nonlinear_friction_coefficient_in_yaw", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        
        # =========================
        # Environmental Configuration (parameters, fixed)
        # =========================
        self.register_variable(Real("rho_seawater", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("rho_air", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("front_above_water_height", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("side_above_water_height", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("cx", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("cy", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("cn", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        
        # =========================
        # Initial states (parameters, fixed)
        # =========================
        self.register_variable(Real("initial_north_position_m", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("initial_east_position_m", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("initial_yaw_angle_rad", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("initial_forward_speed_m_per_s", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("initial_sideways_speed_m_per_s", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("initial_yaw_rate_rad_per_s", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))

        # =========================
        # Input Variables
        # =========================
        self.register_variable(Real("thrust_force", causality=Fmi2Causality.input))
        self.register_variable(Real("rudder_force_v", causality=Fmi2Causality.input))
        self.register_variable(Real("rudder_force_r", causality=Fmi2Causality.input))
        self.register_variable(Real("wind_speed", causality=Fmi2Causality.input))
        self.register_variable(Real("wind_dir_rad", causality=Fmi2Causality.input))
        self.register_variable(Real("current_speed", causality=Fmi2Causality.input))
        self.register_variable(Real("current_dir_rad", causality=Fmi2Causality.input))

        # =========================
        # Output Variables
        # =========================
        self.register_variable(Real("north", causality=Fmi2Causality.output))
        self.register_variable(Real("east", causality=Fmi2Causality.output))
        self.register_variable(Real("yaw_angle_rad", causality=Fmi2Causality.output))
        self.register_variable(Real("forward_speed", causality=Fmi2Causality.output))
        self.register_variable(Real("sideways_speed", causality=Fmi2Causality.output))
        self.register_variable(Real("yaw_rate", causality=Fmi2Causality.output))
        
        self.register_variable(Real("measured_ship_speed", causality=Fmi2Causality.output))
        
        self.register_variable(Real("d_north", causality=Fmi2Causality.output))
        self.register_variable(Real("d_east", causality=Fmi2Causality.output))
        self.register_variable(Real("d_yaw_angle_rad", causality=Fmi2Causality.output))
        self.register_variable(Real("d_forward_speed", causality=Fmi2Causality.output))
        self.register_variable(Real("d_sideways_speed", causality=Fmi2Causality.output))
        self.register_variable(Real("d_yaw_rate", causality=Fmi2Causality.output))
        
        
    def _eps(self):
        return 1e-9
        
    # Save Division Module
    def _safe_div(self, num, den, name=""):
        den = float(den)
        if abs(den) < self._eps():
            raise ValueError(f"Division by ~0 in {name}: den={den}")
        return float(num) / den 
    
    
    def _validate_params(self):
        # Anything that appears in a denominator or matrix must be > 0
        must_be_pos = [
            ("coefficient_of_deadweight_to_displacement", self.coefficient_of_deadweight_to_displacement),
            ("rho_seawater", self.rho_seawater),
            ("length_of_ship", self.length_of_ship),
            ("width_of_ship", self.width_of_ship),
            ("mass_over_linear_friction_coefficient_in_surge", self.mass_over_linear_friction_coefficient_in_surge),
            ("mass_over_linear_friction_coefficient_in_sway", self.mass_over_linear_friction_coefficient_in_sway),
            ("mass_over_linear_friction_coefficient_in_yaw", self.mass_over_linear_friction_coefficient_in_yaw),
        ]
        for n, v in must_be_pos:
            if float(v) <= 0.0:
                raise ValueError(f"Parameter '{n}' must be > 0, got {v}")

        # optional: sanity checks
        if self.dead_weight_tonnage <= 0:
            raise ValueError(f"dead_weight_tonnage must be > 0, got {self.dead_weight_tonnage}")

    
    def pre_compute_initial_variables(self):
        self._validate_params()
        
        # Ship
        self.payload            = 0.9 * (self.dead_weight_tonnage - self.bunkers)
        
        # lsw = DWT/c - DWT  (c must be >0)
        self.lsw                = self._safe_div(self.dead_weight_tonnage, self.coefficient_of_deadweight_to_displacement,
                                                 "lsw: dead_weight_tonnage/coefficient") - self.dead_weight_tonnage
        
        self.mass               = self.lsw + self.payload + self.bunkers + self.ballast
        self.l_ship             = self.length_of_ship
        self.w_ship             = self.width_of_ship
        
        # draft t_ship = m/(rho*L*W)
        denom                   = self.rho_seawater * self.l_ship * self.w_ship
        self.t_ship             = self._safe_div(self.mass, denom, "t_ship")
        
        self.x_g                = 0.0
        self.i_z                = self.mass * (self.l_ship ** 2 + self.w_ship ** 2) / 12.0
        self.proj_area_f        = self.w_ship * self.front_above_water_height
        self.proj_area_l        = self.l_ship * self.side_above_water_height
        
        self.t_surge            = self.mass_over_linear_friction_coefficient_in_surge
        self.t_sway             = self.mass_over_linear_friction_coefficient_in_sway
        self.t_yaw              = self.mass_over_linear_friction_coefficient_in_yaw
        self.ku                 = self.nonlinear_friction_coefficient_in_surge
        self.kv                 = self.nonlinear_friction_coefficient_in_sway
        self.kr                 = self.nonlinear_friction_coefficient_in_yaw
        
        self.x_du, self.y_dv, self.n_dr = self.set_added_mass(self.added_mass_coefficient_in_surge,
                                                              self.added_mass_coefficient_in_sway,
                                                              self.added_mass_coefficient_in_yaw)
    
    
    def set_added_mass(self, surge_coeff, sway_coeff, yaw_coeff):
        ''' Sets the added mass in surge due to surge motion, sway due
            to sway motion and yaw due to yaw motion according to given coeffs.

            args:
                surge_coeff (float): Added mass coefficient in surge direction due to surge motion
                sway_coeff (float): Added mass coefficient in sway direction due to sway motion
                yaw_coeff (float): Added mass coefficient in yaw direction due to yaw motion
            returns:
                x_du (float): Added mass in surge
                y_dv (float): Added mass in sway
                n_dr (float): Added mass in yaw
        '''
        x_du = self.mass * surge_coeff
        y_dv = self.mass * sway_coeff
        n_dr = self.i_z * yaw_coeff
        return x_du, y_dv, n_dr
        
    def rotation(self, yaw_angle_rad):
        ''' Specifies the rotation matrix for rotations about the z-axis, such that
            "body-fixed coordinates" = rotation x "North-east-down-fixed coordinates" .
        '''
        return np.array([[np.cos(yaw_angle_rad), -np.sin(yaw_angle_rad), 0],
                         [np.sin(yaw_angle_rad), np.cos(yaw_angle_rad), 0],
                         [0, 0, 1]])
        

    def mass_matrix(self):
        return np.array([[self.mass + self.x_du, 0, 0],
                         [0, self.mass + self.y_dv, self.mass * self.x_g],
                         [0, self.mass * self.x_g, self.i_z + self.n_dr]])


    def coriolis_matrix(self, forward_speed, sideways_speed, yaw_rate):
        return np.array([[0, 0, -self.mass * (self.x_g * yaw_rate + sideways_speed)],
                         [0, 0, self.mass * forward_speed],
                         [self.mass * (self.x_g * yaw_rate + sideways_speed),
                          -self.mass * forward_speed, 0]])


    def coriolis_added_mass_matrix(self, u_r, v_r):
        return np.array([[0, 0, self.y_dv * v_r],
                        [0, 0, -self.x_du * u_r],
                        [-self.y_dv * v_r, self.x_du * u_r, 0]])


    def linear_damping_matrix(self):
        return np.array([[self.mass / self.t_surge, 0, 0],
                      [0, self.mass / self.t_sway, 0],
                      [0, 0, self.i_z / self.t_yaw]])


    def non_linear_damping_matrix(self, forward_speed, sideways_speed, yaw_rate):
        return np.array([[self.ku * forward_speed, 0, 0],
                       [0, self.kv * sideways_speed, 0],
                       [0, 0, self.kr * yaw_rate]])
        

    def get_wind_force(self, wind_speed, wind_dir_rad, yaw_angle_rad, forward_speed, sideways_speed):
        ''' This method calculates the forces due to the relative
            wind speed, acting on the ship in surge, sway and yaw
            direction.

            :return: Wind force acting in surge, sway and yaw
        '''
        # Unpack wind_args
        uw = wind_speed * np.sin(wind_dir_rad - yaw_angle_rad)
        vw = wind_speed * np.cos(wind_dir_rad - yaw_angle_rad)
        u_rw = uw - forward_speed
        v_rw = vw - sideways_speed
        gamma_rw = -np.arctan2(v_rw, u_rw)
        wind_rw2 = u_rw ** 2 + v_rw ** 2
        c_x = -self.cx * np.sin(gamma_rw)
        c_y = self.cy * np.cos(gamma_rw)
        c_n = self.cn * np.sin(2 * gamma_rw)
        tau_coeff = 0.5 * self.rho_air * wind_rw2
        tau_u = tau_coeff * c_x * self.proj_area_f
        tau_v = tau_coeff * c_y * self.proj_area_l
        tau_n = tau_coeff * c_n * self.proj_area_l * self.l_ship
        return np.array([tau_u, tau_v, tau_n])

        
    def t0hree_dof_kinematics(self, yaw_angle_rad, forward_speed, sideways_speed, yaw_rate):
        ''' Updates the time differientials of the north position, east
            position and yaw angle. Should be called in the simulation
            loop before the integration step.
        '''
        vel = np.array([forward_speed, sideways_speed, yaw_rate])
        dx = np.dot(self.rotation(yaw_angle_rad), vel)
        d_north = dx[0]
        d_east = dx[1]
        d_yaw_angle_rad = dx[2]
        return d_north, d_east, d_yaw_angle_rad
    
    
    def three_dof_kinetics(self, 
                           yaw_angle_rad,
                           forward_speed,
                           sideways_speed,
                           yaw_rate,
                           thrust_force, 
                           rudder_force_v,
                           rudder_force_r,
                           wind_force,
                           current_velocity,
                           *args, 
                           **kwargs):
        ''' Calculates accelerations of the ship, as a function
            of thrust-force, rudder angle, wind forces and the
            states in the previous time-step.
        '''
        # Forces acting
        ctrl_force = np.array([thrust_force, rudder_force_v, rudder_force_r])
        
        # assembling state vector
        vel = np.array([forward_speed, sideways_speed, yaw_rate])

        # Transforming current velocity to ship frame
        v_c = np.dot(np.linalg.inv(self.rotation(yaw_angle_rad)), current_velocity)
        u_r = forward_speed - v_c[0]
        v_r = sideways_speed - v_c[1]

        # Kinetic equation
        M   = self.mass_matrix()
        rhs = (
                - self.coriolis_matrix(forward_speed, sideways_speed, yaw_rate) @ vel
                - self.coriolis_added_mass_matrix(u_r=u_r, v_r=v_r) @ (vel - v_c)
                - (self.linear_damping_matrix() + self.non_linear_damping_matrix(forward_speed, sideways_speed, yaw_rate)) @ (vel - v_c)
                + wind_force + ctrl_force
            )
        dx = np.linalg.solve(M, rhs)
        d_forward_speed = dx[0]
        d_sideways_speed = dx[1]
        d_yaw_rate = dx[2]
        
        return d_forward_speed, d_sideways_speed, d_yaw_rate
        

    def do_step(self, current_time: float, step_size: float) -> bool:
        try:
            # Precompute once
            if not self._precomputed:
                ## Pre-compute initial variables
                self.pre_compute_initial_variables()
                self._precomputed = True
            
            ## Get current ship states
            north               = self.initial_north_position_m
            east                = self.initial_east_position_m
            yaw_angle_rad       = self.initial_yaw_angle_rad
            forward_speed       = self.initial_forward_speed_m_per_s
            sideways_speed      = self.initial_sideways_speed_m_per_s
            yaw_rate            = self.initial_yaw_rate_rad_per_s
            
            ## Get force inputs
            thrust_force        = self.thrust_force
            rudder_force_v      = self.rudder_force_v
            rudder_force_r      = self.rudder_force_r
            
            # Get environment load arguments
            wind_speed          = self.wind_speed
            wind_dir_rad        = self.wind_dir_rad
            current_speed       = self.current_speed
            current_dir_rad     = self.current_dir_rad
            
            # Get force
            wind_force          = self.get_wind_force(wind_speed, wind_dir_rad, yaw_angle_rad, forward_speed, sideways_speed)
            current_velocity    = np.array([current_speed * np.cos(current_dir_rad),
                                            current_speed * np.sin(current_dir_rad),
                                            0.0])
            
            # Compute the kinematics and kinetics
            d_north, d_east, d_yaw_angle_rad          = self.three_dof_kinematics(yaw_angle_rad,
                                                                                  forward_speed, 
                                                                                  sideways_speed, 
                                                                                  yaw_rate)
            d_forward_speed, d_sideways_speed, d_yaw_rate   = self.three_dof_kinetics(yaw_angle_rad,
                                                                                      forward_speed, 
                                                                                      sideways_speed, 
                                                                                      yaw_rate,
                                                                                      thrust_force, 
                                                                                      rudder_force_v,
                                                                                      rudder_force_r,
                                                                                      wind_force,
                                                                                      current_velocity)
            
            ## Get Output
            # Compute d_x
            self.d_north                        = d_north
            self.d_east                         = d_east
            self.d_yaw_angle_rad                = d_yaw_angle_rad
            self.d_forward_speed                = d_forward_speed
            self.d_sideways_speed               = d_sideways_speed
            self.d_yaw_rate                     = d_yaw_rate
            
            # Integrate d_x (Euler integration)
            self.north                          = north          + step_size * d_north
            self.east                           = east           + step_size * d_east
            self.yaw_angle_rad                  = yaw_angle_rad  + step_size * d_yaw_angle_rad
            self.forward_speed                  = forward_speed  + step_size * d_forward_speed
            self.sideways_speed                 = sideways_speed + step_size * d_sideways_speed
            self.yaw_rate                       = yaw_rate       + step_size * d_yaw_rate
            
            # Get the measured speed
            self.measured_ship_speed            = np.sqrt(self.forward_speed ** 2 + self.sideways_speed ** 2)
            
            # Re-assigns the final states to the previous states
            self.initial_north_position_m       = self.north
            self.initial_east_position_m        = self.east
            self.initial_yaw_angle_rad          = self.yaw_angle_rad
            self.initial_forward_speed_m_per_s  = self.forward_speed
            self.initial_sideways_speed_m_per_s = self.sideways_speed
            self.initial_yaw_rate_rad_per_s     = self.yaw_rate
            
        except Exception as e:
            # IMPORTANT: do not crash host
            print(f"[ShipModel] ERROR t={current_time} dt={step_size}: {type(e).__name__}: {e}")
            print(traceback.format_exc())

            # Freeze dynamics safely (keep last state/outputs)
            self.d_north = self.d_east = self.d_yaw_angle_rad = 0.0
            self.d_forward_speed = self.d_sideways_speed = self.d_yaw_rate = 0.0
            # keep measured speed consistent with current outputs
            self.measured_ship_speed = float(np.hypot(self.forward_speed, self.sideways_speed)) if np.isfinite(self.forward_speed) else 0.0
        
        return True