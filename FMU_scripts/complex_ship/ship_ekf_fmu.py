"""
Extended Kalman Filter Observer Python FMU implementation.

Authors : Andreas R.G. Sitorus
Date    : September 2026
"""

from pythonfmu import Fmi2Causality, Fmi2Slave, Fmi2Variability, Real, Integer, Boolean, String
import numpy as np
import sympy as sp
import traceback

class ShipEKF(Fmi2Slave):
    author = "Andreas R.G. Sitorus"
    description = "Extended Kalman Filter Observer Python FMU Implementation for Ship in Transit Co-simulation."
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        
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
        self.rho_seawater                                   = 1025.0
        
        # Initial uncertainty
        self.sigma_p0_n                 = 2.0
        self.sigma_p0_e                 = 2.0
        self.sigma_p0_psi               = 0.05
        self.sigma_p0_u                 = 0.05
        self.sigma_p0_v                 = 0.05
        self.sigma_p0_r                 = 0.05
        
        # Sensors uncertainty
        self.sigma_gps_north            = 2.0               # m
        self.sigma_gps_east             = 2.0               # m
        self.sigma_gyro                 = 0.05              # deg
        self.sigma_speed_log            = 0.05              # m/s
        
        # Prediction uncertainty
        self.sigma_q_n                  = 2.0
        self.sigma_q_e                  = 2.0
        self.sigma_q_psi                = 0.05
        self.sigma_q_u                  = 0.05
        self.sigma_q_v                  = 0.05
        self.sigma_q_r                  = 0.05
        
        ## Input
        self.gps_valid                  = True
        self.gyro_valid                 = True
        self.speed_log_valid            = True
        
        self.tau_u                      = 0.0
        self.tau_v                      = 0.0
        self.tau_r                      = 0.0
        
        self.measured_north             = 0.0
        self.measured_east              = 0.0
        self.measured_heading           = 0.0
        self.measured_ship_speed        = 0.0
        
        ## Output
        self.estimated_n                = 0.0
        self.estimated_e                = 0.0
        self.estimated_psi              = 0.0
        self.estimated_u                = 0.0
        self.estimated_v                = 0.0
        self.estimated_r                = 0.0
        
        ## Internal variables
        self._precomputed               = False
        # State
        self.x                          = None
        # Initial covariance
        self.P                          = None
        # Measurement covariance
        self.R                          = None
        self.R_gps                      = None
        self.R_gyro                     = None
        self.R_speed_log                = None
        # Temporary/simple process covariance
        self.Q                          = None
        # Measurement Jacobian
        self.H                          = None
        
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
        self.register_variable(Real("rho_seawater", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        
        self.register_variable(Real("sigma_gps_north", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("sigma_gps_east", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("sigma_gyro", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("sigma_speed_log", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("sigma_q_n", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("sigma_q_e", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("sigma_q_psi", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("sigma_q_u", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("sigma_q_v", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("sigma_q_r", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("sigma_p0_n", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("sigma_p0_e", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("sigma_p0_psi", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("sigma_p0_u", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("sigma_p0_v", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("sigma_p0_r", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        
        # =========================
        # Input
        # =========================
        self.register_variable(Boolean("gps_valid", causality=Fmi2Causality.input, variability=Fmi2Variability.discrete))
        self.register_variable(Boolean("gyro_valid", causality=Fmi2Causality.input, variability=Fmi2Variability.discrete))
        self.register_variable(Boolean("speed_log_valid", causality=Fmi2Causality.input, variability=Fmi2Variability.discrete))
        self.register_variable(Real("tau_u", causality=Fmi2Causality.input))
        self.register_variable(Real("tau_v", causality=Fmi2Causality.input))
        self.register_variable(Real("tau_r", causality=Fmi2Causality.input))
        self.register_variable(Real("measured_north", causality=Fmi2Causality.input))
        self.register_variable(Real("measured_east", causality=Fmi2Causality.input))
        self.register_variable(Real("measured_heading", causality=Fmi2Causality.input))
        self.register_variable(Real("measured_ship_speed", causality=Fmi2Causality.input))

        # =========================
        # Output
        # =========================
        self.register_variable(Real("estimated_n", causality=Fmi2Causality.output))
        self.register_variable(Real("estimated_e", causality=Fmi2Causality.output))
        self.register_variable(Real("estimated_psi", causality=Fmi2Causality.output))
        self.register_variable(Real("estimated_u", causality=Fmi2Causality.output))
        self.register_variable(Real("estimated_v", causality=Fmi2Causality.output))
        self.register_variable(Real("estimated_r", causality=Fmi2Causality.output))
        
        
    def _wrap_to_pi(self, a):
        return (a + np.pi) % (2*np.pi) - np.pi
    
    def _safe_div(self, num, den, name=""):
        den = float(den)
        if abs(den) < 1e-9:
            raise ValueError(f"Division by ~0 in {name}: den={den}")
        return float(num) / den 
    
    def _compute_ship_parameters(self):
        # Ship
        self.payload            = 0.9 * (self.dead_weight_tonnage - self.bunkers)
        
        # lsw = DWT/c - DWT  (c must be >0)
        self.lsw                = self._safe_div(self.dead_weight_tonnage, self.coefficient_of_deadweight_to_displacement,
                                                 "lsw: dead_weight_tonnage/coefficient") - self.dead_weight_tonnage
        
        self.mass               = self.lsw + self.payload + self.bunkers + self.ballast
        self.l_ship             = self.length_of_ship
        self.w_ship             = self.width_of_ship
        
        denom                   = self.rho_seawater * self.l_ship * self.w_ship
        self.t_ship             = self._safe_div(self.mass, denom, "t_ship")
        
        self.x_g                = 0.0
        self.i_z                = self.mass * (self.l_ship ** 2 + self.w_ship ** 2) / 12.0
        
        self.t_surge            = self.mass_over_linear_friction_coefficient_in_surge
        self.t_sway             = self.mass_over_linear_friction_coefficient_in_sway
        self.t_yaw              = self.mass_over_linear_friction_coefficient_in_yaw
        self.ku                 = self.nonlinear_friction_coefficient_in_surge
        self.kv                 = self.nonlinear_friction_coefficient_in_sway
        self.kr                 = self.nonlinear_friction_coefficient_in_yaw
        
        # Added Mass
        self.am_u               = self.mass * self.added_mass_coefficient_in_surge
        self.am_v               = self.mass * self.added_mass_coefficient_in_sway
        self.am_r               = self.i_z  * self.added_mass_coefficient_in_yaw
    
    def control_plant_model(self, step_size):
        """
            Low-frequency control plant model
        """
        
        ## Symbols
        # State symbol
        n, e, psi, u, v, r              = sp.symbols('n, e, psi, u, v, r', real=True)
        
        # Input symbol
        tau_u, tau_v, tau_r             = sp.symbols('tau_u, tau_v, tau_r', real=True)
        
        ## Vectors
        eta     = sp.Matrix([n, e, psi])
        nu      = sp.Matrix([u, v, r])
        x       = sp.Matrix([n, e, psi, u, v, r])
        tau     = sp.Matrix([tau_u, tau_v, tau_r])
        
        ## Kinematic Equation
        # Ship parameters
        self._compute_ship_parameters()
        
        # Rotaion matrix
        R_psi   = sp.Matrix([
            [sp.cos(psi), -sp.sin(psi), 0],
            [sp.sin(psi), sp.cos(psi), 0],
            [0, 0, 1]
        ])
        
        # Integration
        eta_dot     = R_psi @ nu
        eta_next    = eta + eta_dot * step_size
        
        ## Kinetic Equation
        # Rigid Body Mass and Added Mass matrix
        MARB = sp.Matrix([
            [self.mass + self.am_u , 0 , 0],
            [0 , self.mass + self.am_v , self.mass * self.x_g ],
            [0 , self.mass * self.x_g , self.i_z + self.am_r]
        ])
        
        # Coriolis Matrix
        CRB = sp.Matrix([
            [0, 0, -self.mass * (self.x_g * r + v)],
            [0, 0, self.mass * u],
            [self.mass * (self.x_g * r + v), -self.mass * u, 0]
        ])
        
        # Coriolis Added Mass Matrix
        CARB = sp.Matrix([
            [0, 0, self.am_v * v],
            [0, 0, -self.am_u * u],
            [-self.am_v * v , self.am_u * u, 0]
        ])
        
        # Linear Damping Matrix
        DL = sp.Matrix([
            [self.mass / self.t_surge, 0, 0],
            [0, self.mass / self.t_sway, 0],
            [0, 0, self.i_z / self.t_yaw]
        ])
        
        # Non Linear Damping Matrix
        DNL = sp.Matrix([
            [self.ku * sp.Abs(u), 0, 0],
            [0, self.kv * sp.Abs(v), 0],
            [0, 0, self.kr * sp.Abs(r)]
        ])
        
        # Integration
        nu_dot  = MARB.inv() @ (tau - (CRB + CARB) @ nu - (DL + DNL) @ nu)
        nu_next = nu + nu_dot * step_size

        ## Complete dynamics
        # State model
        f = sp.Matrix([
            eta_next[0, 0],     # n
            eta_next[1, 0],     # e
            eta_next[2, 0],     # psi
            nu_next[0, 0],      # u
            nu_next[1, 0],      # v
            nu_next[2, 0],      # r
        ])
        
        # Measurement model
        h = sp.Matrix([
            n, 
            e, 
            psi, 
            u
        ])
        
        ## Compute the Jacobian
        J_x     = f.jacobian(x)     # State Jacobian
        H_x     = h.jacobian(x)      # Measurement Jacobian
        
        ## Convert to lambdified function
        self.J_x_lambda     = sp.lambdify([n, e, psi, u, v, r], J_x, "numpy")
        self.H_x_lambda     = sp.lambdify([n, e, psi, u, v, r], H_x, "numpy")
        self.f_lambda       = sp.lambdify([n, e, psi, u, v, r] + [tau_u, tau_v, tau_r], f, "numpy")
        self.h_lambda       = sp.lambdify([n, e, psi, u, v, r], h, "numpy")
       
    def f(self, x:np.ndarray, u:np.ndarray, *args, **kwargs) -> np.ndarray:
        """
        System model: x' = f(x, u)
        """
        return np.asarray(self.f_lambda(*(x.tolist() + u.tolist()))).reshape(-1)
    
    def dfdx(self, x:np.ndarray, *args, **kwargs) -> np.ndarray:
        """
        Jacobian of system model: df/dx for x = x_prev, u = u_prev
        """
        return np.asarray(self.J_x_lambda(*x.tolist()), dtype=float)
    
    def h(self, x:np.ndarray, *args, **kwargs) -> np.ndarray:
        """
        Measurement model: z = h(x)
        """
        return np.asarray(self.h_lambda(*x.tolist()), dtype=float).squeeze()
    
    def dhdx(self, x:np.ndarray, *args, **kwargs) -> np.ndarray:
        """
        Jacobian of the measurement model: dh/dx for z = h(x)
        """
        return np.asarray(self.H_x_lambda(*x.tolist()), dtype=float)
    
    def get_PQR(self):
        ## P Matrix
        # For 6 states: n, e, psi, u, v, r
        self.P  = np.diag([
            self.sigma_p0_n ** 2,
            self.sigma_p0_e ** 2,
            np.deg2rad(self.sigma_p0_psi) ** 2,
            self.sigma_p0_u ** 2,
            self.sigma_p0_v ** 2,
            self.sigma_p0_r ** 2,
        ])
        
        ## Q Matrix
        self.Q  = np.diag([
            self.sigma_q_n ** 2,
            self.sigma_q_e ** 2,
            np.deg2rad(self.sigma_q_psi) ** 2,
            self.sigma_q_u ** 2,
            self.sigma_q_v ** 2,
            self.sigma_q_r ** 2,
        ])
        
        ## R Matrix
        # Sensor Covariance
        self.R  = np.diag([
            self.sigma_gps_north **2,
            self.sigma_gps_east **2,
            np.deg2rad(self.sigma_gyro) ** 2,
            self.sigma_speed_log ** 2
        ])
        
        # GPS Covariance
        self.R_gps  = np.diag([
            self.sigma_gps_north **2,
            self.sigma_gps_east **2,
        ])
        
        # Gyro Covariance
        self.R_gyro  = np.array([
            [np.deg2rad(self.sigma_gyro) ** 2]
        ])
        
        # Speed Log Covariance
        self.R_speed_log  = np.array([
            [self.sigma_speed_log ** 2]
        ])
    
    def predict(self, tau:np.ndarray) -> np.ndarray:
        # Get Jacobian
        dfdx    = self.dfdx(self.x, tau)
        
        # Propagates the covariance
        self.P  = dfdx @ self.P @ dfdx.T + self.Q
        
        # Predict states
        self.x  = self.f(self.x, tau)
    
    def _update(self, z:np.ndarray, h:np.ndarray, dhdx:np.ndarray, R, angle_indices=None) -> np.ndarray:
        # Inovation -> Residuals between measurement and prediction
        y           = z - h
        
        # Wrap inovation with angle
        if angle_indices:
            for idx in angle_indices:
                y[idx] = self._wrap_to_pi(y[idx])
        
        # Residual Covariance -> Expected combined uncertainty of of prediction & measurement
        S           = dhdx @ self.P @ dhdx.T + R
        
        # Kalman Gain -> Balance factor for blending prediction and measurement
        # K           = self.P @ dhdx.T @ np.linalg.inv(S)
        K           = np.linalg.solve(S.T, (self.P @ dhdx.T).T).T
        
        # Correct the state estimate
        self.x      = self.x + K @ y
        
        # For psi, wrap to pi
        self.x[2]   = self._wrap_to_pi(self.x[2])
        
        # Correct the Covariance
        I           = np.eye(self.P.shape[0])
        IKH         = I - K @ dhdx
        self.P      = IKH @ self.P @ IKH.T + K @ R @ K.T
        
        return self.x
    
    def update_gps(self, north, east):
        z = np.array([
            north,
            east
        ])
        
        h = np.array([
            self.x[0],
            self.x[1]
        ])
        
        dhdx = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0]
        ])
        
        self._update(z=z, h=h, dhdx=dhdx, R=self.R_gps)
        
    def update_gyro(self, heading):
        z = np.array([
            heading
        ])
        
        h = np.array([
            self.x[2]
        ])
        
        dhdx = np.array([
            [0, 0, 1, 0, 0, 0]
        ])
        
        self._update(z=z, h=h, dhdx=dhdx, R=self.R_gyro, angle_indices=[0])
        
    def update_speed_log(self, speed):
        z = np.array([
            speed
        ])
        
        h = np.array([
            self.x[3]
        ])
        
        dhdx = np.array([
            [0, 0, 0, 1, 0, 0]
        ])
        
        self._update(z=z, h=h, dhdx=dhdx, R=self.R_speed_log)
    
    def do_step(self, current_time: float, step_size: float) -> bool:
        try:
            if not self._precomputed:
                # Get P, Q, and R Matrix
                self.get_PQR()
                
                # Set the initial x
                self.x = np.array([
                    self.measured_north,
                    self.measured_east,
                    self._wrap_to_pi(self.measured_heading),
                    self.measured_ship_speed,
                    0.0,                                        # Sway speed assume to be 0.0
                    0.0                                         # Yaw rate assume to be 0.0
                ])
                
                # Set the control plant model
                self.control_plant_model(step_size)
                
                # Turn off precomputed
                self._precomputed = True
                
            
            # Known generalized forces
            tau = np.array([
                self.tau_u,
                self.tau_v,
                self.tau_r
            ])
            
            # ========================================
            # EKF prediction
            # ========================================
            self.predict(tau=tau)
            
            # ========================================
            # Sensor corrections
            # ========================================
            if self.gps_valid:
                self.update_gps(
                    self.measured_north,
                    self.measured_east
                )
            
            if self.gyro_valid:
                self.update_gyro(
                    self.measured_heading
                )
            
            if self.speed_log_valid:
                self.update_speed_log(
                    self.measured_ship_speed
                )
                
            # ========================================
            # Observer outputs
            # ========================================
            self.estimated_n    = self.x[0]
            self.estimated_e    = self.x[1]
            self.estimated_psi  = self.x[2]
            self.estimated_u    = self.x[3]
            self.estimated_v    = self.x[4]
            self.estimated_r    = self.x[5]
                
        except Exception as e:
            # IMPORTANT: do not crash host
            print(f"[ShipEKF] ERROR t={current_time} dt={step_size}: {type(e).__name__}: {e}")
            print(traceback.format_exc())
            
            # Freeze dynamics safely (keep last state/outputs)
            self.estimated_n    = self.x[0]
            self.estimated_e    = self.x[1]
            self.estimated_psi  = self.x[2]
            self.estimated_u    = self.x[3]
            self.estimated_v    = self.x[4]
            self.estimated_r    = self.x[5]
        
        return True