"""
Extended Kalman Filter Observer for Target Ship Tracking Python FMU implementation.

Authors : Andreas R.G. Sitorus
Date    : September 2026
"""

from pythonfmu import Fmi2Causality, Fmi2Slave, Fmi2Variability, Real, Integer, Boolean, String
import numpy as np
import sympy as sp
import traceback

class TargetShipTrackingEKF(Fmi2Slave):
    author = "Andreas R.G. Sitorus"
    description = "Extended Kalman Filter Observer for Target Ship Tracking  Python FMU Implementation for Ship in Transit Co-simulation."
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        
        ## Parameters
        # Ship configuration
        self.max_dt                             = 5.0
        self.length_of_ship                     = 0.0
        self.width_of_ship                      = 0.0
        
        # Unknwon acceleration uncertainty
        self.sigma_acc_north                    = 0.2   # m/s2
        self.sigma_acc_east                     = 0.2   # m/s2
        
        # Own ship initial states
        self.initial_measured_own_north         = 0.0
        self.initial_measured_own_east          = 0.0
        self.initial_measured_own_yaw_angle     = 0.0
        self.initial_measured_own_speed_north   = 0.0
        self.initial_measured_own_speed_east    = 0.0
        
        # Target ship initial states
        self.initial_measured_tar_north         = 0.0
        self.initial_measured_tar_east          = 0.0
        self.initial_measured_tar_speed_north   = 0.0
        self.initial_measured_tar_speed_east    = 0.0
        
        # Initial uncertainty
        self.sigma_p0_n                         = 10.0
        self.sigma_p0_e                         = 10.0
        self.sigma_p0_vn                        = 0.2
        self.sigma_p0_ve                        = 0.2
        
        ## Sensors uncertainty
        # Radar
        self.sigma_radar_range                  = 10.0
        self.sigma_radar_range_rate             = 0.2
        self.sigma_radar_bearing_deg            = 1.0
        
        # Fore Camera
        self.sigma_fore_camera_range            = 50.0
        self.sigma_fore_camera_bearing_deg      = 0.1
        
        # Aft Camera
        self.sigma_aft_camera_range             = 50.0
        self.sigma_aft_camera_bearing_deg       = 0.1
        
        # Port Camera
        self.sigma_port_camera_range            = 50.0
        self.sigma_port_camera_bearing_deg      = 0.1
        
        # Starboard Camera
        self.sigma_starboard_camera_range       = 50.0
        self.sigma_starboard_camera_bearing_deg = 0.1
        
        ## Input
        # Own ship states
        self.measured_own_north                 = 0.0
        self.measured_own_east                  = 0.0
        self.measured_own_yaw_angle             = 0.0
        self.measured_own_speed_north           = 0.0
        self.measured_own_speed_east            = 0.0
        
        # Target ship states
        self.measured_tar_north                 = 0.0
        self.measured_tar_east                  = 0.0
        self.measured_tar_speed_north           = 0.0
        self.measured_tar_speed_east            = 0.0
        
        # Measurement validity flags
        self.radar_valid                        = True
        self.camera_fore_valid                  = True
        self.camera_aft_valid                   = True
        self.camera_port_valid                  = True
        self.camera_starboard_valid             = True
        
        # Radar
        self.radar_detection                    = True
        self.radar_measured_range               = 0.0
        self.radar_measured_range_rate          = 0.0
        self.radar_measured_bearing             = 0.0
        
        # Fore Camera
        self.fore_camera_detection              = True
        self.fore_camera_measured_range         = 0.0
        self.fore_camera_measured_bearing       = 0.0
        
        # Aft Camera
        self.aft_camera_detection               = True
        self.aft_camera_measured_range          = 0.0
        self.aft_camera_measured_bearing        = 0.0
        
        # Port Camera
        self.port_camera_detection              = True
        self.port_camera_measured_range         = 0.0
        self.port_camera_measured_bearing       = 0.0
        
        # Starboard Camera
        self.starboard_camera_detection         = True
        self.starboard_camera_measured_range    = 0.0
        self.starboard_camera_measured_bearing  = 0.0
        
        ## Output
        self.estimated_n                        = 0.0
        self.estimated_e                        = 0.0
        self.estimated_vn                       = 0.0
        self.estimated_ve                       = 0.0
        
        ## Internal variables
        self._precomputed                       = False
        # State
        self.x                                  = None
        # Initial covariance
        self.P                                  = None
        # Measurement covariance
        self.R                                  = None
        self.R_radar                            = None
        self.R_fore_camera                      = None
        self.R_aft_camera                       = None
        self.R_port_camera                      = None
        self.R_starboard_camera                 = None
        # Temporary/simple process covariance
        self.Q                                  = None
        # Measurement Jacobian
        self.H                                  = None
        # For local integration
        self.n_sub                              = 1
        self.dt_internal                        = None
        
        ## Registration
        # =========================
        # Ship configuration (parameters, fixed)
        # =========================
        self.register_variable(Real("max_dt", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("length_of_ship", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("width_of_ship", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        
        # Unknown acceleration uncertainty
        self.register_variable(Real("sigma_acc_north", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("sigma_acc_east", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        
        # Initial P uncertainty
        self.register_variable(Real("sigma_p0_n", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("sigma_p0_e", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("sigma_p0_vn", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("sigma_p0_ve", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        
        # Sensors uncertainty
        self.register_variable(Real("sigma_radar_range", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("sigma_radar_range_rate", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("sigma_radar_bearing_deg", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("sigma_fore_camera_range", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("sigma_fore_camera_bearing_deg", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("sigma_aft_camera_range", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("sigma_aft_camera_bearing_deg", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("sigma_port_camera_range", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("sigma_port_camera_bearing_deg", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("sigma_starboard_camera_range", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("sigma_starboard_camera_bearing_deg", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        
        # Own ship initial measurement
        self.register_variable(Real("initial_measured_own_north", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("initial_measured_own_east", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("initial_measured_own_yaw_angle", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("initial_measured_own_speed_north", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("initial_measured_own_speed_east", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        
        # Target ship initial measurement
        self.register_variable(Real("initial_measured_tar_north", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("initial_measured_tar_east", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("initial_measured_tar_speed_north", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("initial_measured_tar_speed_east", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        
        # =========================
        # Input
        # =========================
        # Own ship states
        self.register_variable(Real("measured_own_north", causality=Fmi2Causality.input))
        self.register_variable(Real("measured_own_east", causality=Fmi2Causality.input))
        self.register_variable(Real("measured_own_yaw_angle", causality=Fmi2Causality.input))
        self.register_variable(Real("measured_own_speed_north", causality=Fmi2Causality.input))
        self.register_variable(Real("measured_own_speed_east", causality=Fmi2Causality.input))
        
        # Target ship states
        self.register_variable(Real("measured_tar_north", causality=Fmi2Causality.input))
        self.register_variable(Real("measured_tar_east", causality=Fmi2Causality.input))
        self.register_variable(Real("measured_tar_speed_north", causality=Fmi2Causality.input))
        self.register_variable(Real("measured_tar_speed_east", causality=Fmi2Causality.input))
        
        # Measurement validity flags
        self.register_variable(Boolean("radar_valid", causality=Fmi2Causality.input, variability=Fmi2Variability.discrete))
        self.register_variable(Boolean("camera_fore_valid", causality=Fmi2Causality.input, variability=Fmi2Variability.discrete))
        self.register_variable(Boolean("camera_aft_valid", causality=Fmi2Causality.input, variability=Fmi2Variability.discrete))
        self.register_variable(Boolean("camera_port_valid", causality=Fmi2Causality.input, variability=Fmi2Variability.discrete))
        self.register_variable(Boolean("camera_starboard_valid", causality=Fmi2Causality.input, variability=Fmi2Variability.discrete))
        
        # Radar
        self.register_variable(Boolean("radar_detection", causality=Fmi2Causality.input, variability=Fmi2Variability.discrete))
        self.register_variable(Real("radar_measured_range", causality=Fmi2Causality.input))
        self.register_variable(Real("radar_measured_range_rate", causality=Fmi2Causality.input))
        self.register_variable(Real("radar_measured_bearing", causality=Fmi2Causality.input))
        
        # Fore Camera
        self.register_variable(Boolean("fore_camera_detection", causality=Fmi2Causality.input, variability=Fmi2Variability.discrete))
        self.register_variable(Real("fore_camera_measured_range", causality=Fmi2Causality.input))
        self.register_variable(Real("fore_camera_measured_bearing", causality=Fmi2Causality.input))
        
        # Aft Camera
        self.register_variable(Boolean("aft_camera_detection", causality=Fmi2Causality.input, variability=Fmi2Variability.discrete))
        self.register_variable(Real("aft_camera_measured_range", causality=Fmi2Causality.input))
        self.register_variable(Real("aft_camera_measured_bearing", causality=Fmi2Causality.input))
        
        # Port Camera
        self.register_variable(Boolean("port_camera_detection", causality=Fmi2Causality.input, variability=Fmi2Variability.discrete))
        self.register_variable(Real("port_camera_measured_range", causality=Fmi2Causality.input))
        self.register_variable(Real("port_camera_measured_bearing", causality=Fmi2Causality.input))
        
        # Starboard Camera
        self.register_variable(Boolean("starboard_camera_detection", causality=Fmi2Causality.input, variability=Fmi2Variability.discrete))
        self.register_variable(Real("starboard_camera_measured_range", causality=Fmi2Causality.input))
        self.register_variable(Real("starboard_camera_measured_bearing", causality=Fmi2Causality.input))

        # =========================
        # Output
        # =========================
        self.register_variable(Real("estimated_n", causality=Fmi2Causality.output))
        self.register_variable(Real("estimated_e", causality=Fmi2Causality.output))
        self.register_variable(Real("estimated_vn", causality=Fmi2Causality.output))
        self.register_variable(Real("estimated_ve", causality=Fmi2Causality.output))
        
    def _wrap_to_pi(self, a):
        return (a + np.pi) % (2*np.pi) - np.pi
    
    def control_plant_model(self, step_size):
        """
            Low-frequency control plant model
        """
        ## Symbols
        # State symbol
        # Target ship
        n, e, vn, ve  = sp.symbols('n, e, vn, ve', real=True)
        
        # Own ship
        no, eo, psi_o, vno, veo = sp.symbols(
            'no eo psi_o vno veo',
            real=True
        )
        
        # Camera lever arm and boresight angle
        lx, ly, theta_cam = sp.symbols('lx ly theta_cam', real=True)
        
        # ---------------------------------------------------
        # Target ship model
        # ---------------------------------------------------
        ## Vectors
        eta         = sp.Matrix([n, e])
        nu          = sp.Matrix([vn, ve])
        x           = sp.Matrix([n, e, vn, ve])
        
        ## Kinematic Equation    
        # Integration
        eta_dot     = nu
        eta_next    = eta + eta_dot * step_size
        
        nu_next     = nu

        ## Complete dynamics
        # State model
        f = sp.Matrix([
            eta_next[0, 0],     # n
            eta_next[1, 0],     # e
            nu_next[0, 0],      # u
            nu_next[1, 0],      # v
        ])
        
        ## Compute the Jacobian
        J_x     = f.jacobian(x)      # State Jacobian
        # ---------------------------------------------------
        
        # ---------------------------------------------------
        # Radar model
        # ---------------------------------------------------
        # Position difference
        dn  = n - no
        de  = e - eo
        
        # Velocity difference
        dvn = vn - vno
        dve = ve - veo
        
        # Range
        rho = sp.sqrt(dn**2 + de**2)
        
        # Range rate
        rho_dot = (dn * dvn + de *dve) / rho
        
        # Bearing
        bearing = sp.atan2(de, dn) - psi_o
        
        # Radar measurement model
        h_radar = sp.Matrix([rho, rho_dot, bearing])
        
        # Radar measurement model Jacobian
        H_radar_x = h_radar.jacobian(x)
        # ---------------------------------------------------
        
        # ---------------------------------------------------
        # Camera model
        # ---------------------------------------------------
        # Lever arm
        lever = sp.Matrix([lx, ly])
        
        # NED to Body Rotational matrix
        R_n2b = sp.Matrix([
            [sp.cos(psi_o),  sp.sin(psi_o)],
            [-sp.sin(psi_o), sp.cos(psi_o)]
        ])
        
        # Relative target NED position
        r_ned   = sp.Matrix([
            n - no,
            e - eo
        ])
        
        # Relative target to camera position
        r_cam   = R_n2b @ r_ned - lever
        
        ## Measurement model
        # Range
        rho_cam     = sp.sqrt(r_cam[0]**2 + r_cam[1]**2)
        # Bearing
        beta_cam    = sp.atan2(r_cam[1], r_cam[0]) - theta_cam
        
        h_cam   = sp.Matrix([rho_cam, beta_cam])
        
        H_cam_x     = h_cam.jacobian(x)
        # ---------------------------------------------------
        
        ## Convert to lambdified function
        self.f_lambda           = sp.lambdify([n, e, vn, ve], f, "numpy")
        self.J_x_lambda         = sp.lambdify([n, e, vn, ve], J_x, "numpy")
        
        self.h_radar_lambda     = sp.lambdify([n, e, vn, ve] + [no, eo, psi_o, vno, veo], h_radar, "numpy")
        self.H_radar_x_lambda   = sp.lambdify([n, e, vn, ve], H_radar_x, "numpy")
        
        self.h_cam_lambda       = sp.lambdify([n, e, vn, ve] + [no, eo, psi_o, lx, ly, theta_cam], h_cam, "numpy")
        self.H_cam_x_lambda     = sp.lambdify([n, e, vn, ve], H_cam_x, "numpy")
        
        
    def f(self, x:np.ndarray, *args, **kwargs) -> np.ndarray:
        """
        System model: x' = f(x)
        """
        return np.asarray(self.f_lambda(*x.tolist())).squeeze()
    
    def dfdx(self, x:np.ndarray, *args, **kwargs) -> np.ndarray:
        """
        Jacobian of system model: df/dx for x = x_prev, u = u_prev
        """
        return np.asarray(self.J_x_lambda(*x.tolist()), dtype=float)
    
    def h_radar(self, x:np.ndarray, m_radar:np.ndarray, *args, **kwargs) -> np.ndarray:
        """
        radar model: z = h_radar(x, m_radar)
        """
        return np.asarray(self.h_radar_lambda(*(x.tolist() + m_radar.tolist())), dtype=float).reshape(-1) # Ensure flattened
    
    def dhdx_radar(self, x:np.ndarray, m_radar:np.ndarray, *args, **kwargs) -> np.ndarray:
        """
        Jacobian of the radar model: dh_radar/dx
        """
        return np.asarray(self.H_radar_x_lambda(*(x.tolist() + m_radar.tolist())), dtype=float)
    
    def h_cam(self, x:np.ndarray, m_cam:np.ndarray, *args, **kwargs) -> np.ndarray:
        """
        camera model: z = h_cam(x, m_cam)
        """
        return np.asarray(self.h_cam_lambda(*(x.tolist() + m_cam.tolist())), dtype=float).reshape(-1) # Ensure flattened
    
    def dhdx_cam(self, x:np.ndarray, m_cam:np.ndarray, *args, **kwargs) -> np.ndarray:
        """
        Jacobian of the camera model: dh_radar/dx
        """
        return np.asarray(self.H_cam_x_lambda(*(x.tolist() + m_cam.tolist())), dtype=float)
    
    def get_PQR(self):
        ## P Matrix
        # For 6 states: n, e, vn, ve
        self.P  = np.diag([
            self.sigma_p0_n ** 2,
            self.sigma_p0_e ** 2,
            self.sigma_p0_vn ** 2,
            self.sigma_p0_ve ** 2,
        ])
        
        ## Q Matrix
        # Contains Variance and Covariance
        # Position variance due to unknown acceleration         : \Delta Pos = (1/2 * a * dt**2)**2
        # Speed variance due to unknown acceleration            : \Delta Spd = (a * dt)**2
        # Position-speed covariance due to unknown acceleration : (1/2 * a * dt**2) * (a * dt)
        
        # Unknown variance due to unknown acceleration M(4x2)
        G = np.array([
            [(0.5 * self.dt_internal**2), 0.0],     # North position variance due to unknown acceleration
            [0.0, (0.5 * self.dt_internal**2)],     # East speed variance due to unknown acceleration
            [self.dt_internal, 0.0],                # North position variance due to unknown acceleration
            [0.0, self.dt_internal]                 # East speed variance due to unknown acceleration
        ])
        
        # Unknown acceleration variance M(2x1)
        Q_acc   = np.diag([
            self.sigma_acc_north**2,
            self.sigma_acc_east**2
        ])
        
        # Propagate the unknown acceleration uncertainty 
        # to the Q acceleration matrix
        # M(4x2) @ M(2x2) @ M(2x4) = M(4x4)
        self.Q = G @ Q_acc @ G.T
        
        ## R Matrix
        # Sensor Covariance
        self.R  = np.diag([
            self.sigma_radar_range **2,
            self.sigma_radar_range_rate **2,
            self._wrap_to_pi(np.deg2rad(self.sigma_radar_bearing_deg)) ** 2,
            self.sigma_fore_camera_range ** 2,
            self._wrap_to_pi(np.deg2rad(self.sigma_fore_camera_bearing_deg)) ** 2,
            self.sigma_aft_camera_range ** 2,
            self._wrap_to_pi(np.deg2rad(self.sigma_aft_camera_bearing_deg)) ** 2,
            self.sigma_port_camera_range ** 2,
            self._wrap_to_pi(np.deg2rad(self.sigma_port_camera_bearing_deg)) ** 2,
            self.sigma_starboard_camera_range ** 2,
            self._wrap_to_pi(np.deg2rad(self.sigma_starboard_camera_bearing_deg)) ** 2,
        ])
        
        # Radar Covariance
        self.R_radar  = np.diag([
            self.sigma_radar_range **2,
            self.sigma_radar_range_rate **2,
            self._wrap_to_pi(self.sigma_radar_bearing_deg) ** 2,
        ])
        
        # Fore Camera Covariance
        self.R_fore_camera  = np.diag([
            self.sigma_fore_camera_range ** 2,
            self._wrap_to_pi(self.sigma_fore_camera_bearing_deg) ** 2,
        ])
        
        # Aft Camera Covariance
        self.R_aft_camera  = np.diag([
            self.sigma_aft_camera_range ** 2,
            self._wrap_to_pi(self.sigma_aft_camera_bearing_deg) ** 2,
        ])
        
        # Port Camera Covariance
        self.R_port_camera  = np.diag([
            self.sigma_port_camera_range ** 2,
            self._wrap_to_pi(self.sigma_port_camera_bearing_deg) ** 2,
        ])
        
        # Starboard Camera Covariance
        self.R_starboard_camera  = np.diag([
            self.sigma_starboard_camera_range ** 2,
            self._wrap_to_pi(self.sigma_starboard_camera_bearing_deg) ** 2,
        ])
    
    def predict(self) -> np.ndarray:
        # Get Jacobian
        dfdx    = self.dfdx(self.x)
        
        # Propagates the covariance
        self.P  = dfdx @ self.P @ dfdx.T + self.Q
        
        # Predict states
        self.x  = self.f(self.x)
    
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
        K           = np.linalg.solve(S.T, (self.P @ dhdx.T).T).T
        
        # Correct the state estimate
        self.x      = self.x + K @ y
        
        # Correct the Covariance
        I           = np.eye(self.P.shape[0])
        IKH         = I - K @ dhdx
        self.P      = IKH @ self.P @ IKH.T + K @ R @ K.T
        
        return self.x
    
    def update_radar(self):
        ## Actual measurement
        z = np.array([
            self.radar_measured_range,
            self.radar_measured_range_rate,
            self.radar_measured_bearing
        ])
        
        ## Measurement model
        # no, eo, psi_o, vno, veo
        m_radar = np.array([
            self.measured_own_north,
            self.measured_own_east,
            self.measured_own_yaw_angle,
            self.measured_own_speed_north,
            self.measured_own_speed_east
        ])
        h       = self.h_radar(self.x, m_radar)
        
        ## Measurement model Jacobian
        dhdx = self.dhdx_radar(self.x, m_radar)
        
        self._update(z=z, h=h, dhdx=dhdx, R=self.R_radar, angle_indices=[2])
        
    def update_camera(self, orientation="fore"):
        # Get lever arm
        if orientation == "fore":
            measured_range      = self.fore_camera_measured_range
            measured_bearing    = self.fore_camera_measured_bearing
            lever_arm           = np.array([self.length_of_ship/2, 0.0])
            theta_cam           = 0.0
            R_cam               = self.R_fore_camera
        elif orientation == "aft":
            measured_range      = self.aft_camera_measured_range
            measured_bearing    = self.aft_camera_measured_bearing
            lever_arm           = np.array([-self.length_of_ship/2, 0.0])
            theta_cam           = np.pi
            R_cam               = self.R_aft_camera
        elif orientation == "port":
            measured_range      = self.port_camera_measured_range
            measured_bearing    = self.port_camera_measured_bearing
            lever_arm           = np.array([0.0, -self.width_of_ship/2])
            theta_cam           = -np.pi / 2
            R_cam               = self.R_port_camera
        elif orientation == "starboard":
            measured_range      = self.starboard_camera_measured_range
            measured_bearing    = self.starboard_camera_measured_bearing
            lever_arm           = np.array([0.0, self.width_of_ship/2])
            theta_cam           = np.pi / 2
            R_cam               = self.R_starboard_camera
        
        ## Actual measurement
        z = np.array([
            measured_range,
            measured_bearing
        ])
        
        ## Measurement model
        # no, eo, psi_o, lx, ly, theta_cam
        m_cam = np.array([
            self.measured_own_north,
            self.measured_own_east,
            self.measured_own_yaw_angle,
            lever_arm[0],
            lever_arm[1],
            theta_cam
        ])
        
        h = self.h_cam_lambda(self.x, m_cam)
        
        ## Measurement model Jacobian
        dhdx = self.dhdx_cam(self.x, m_cam)
        
        self._update(z=z, h=h, dhdx=dhdx, R=R_cam, angle_indices=[1])
    
    def do_step(self, current_time: float, step_size: float) -> bool:
        try:
            if not self._precomputed:
                # Determine local integration resolution
                self.n_sub = max(1, int(np.ceil(step_size / self.max_dt)))
                self.dt_internal = step_size / self.n_sub
                
                # Set the initial x
                self.x = np.array([
                    self.initial_measured_tar_north,
                    self.initial_measured_tar_east,
                    self.initial_measured_tar_speed_north,
                    self.initial_measured_tar_speed_east,
                ])
                
                # Get P, Q, and R Matrix
                self.get_PQR()
                
                # Set the control plant model
                self.control_plant_model(self.dt_internal)
                
                # Turn off precomputed
                self._precomputed = True
            
            # ========================================
            # EKF prediction
            # ========================================            
            for i in range(self.n_sub):
                self.predict()
            
            # ========================================
            # Sensor corrections
            # ========================================
            # Radar
            if self.radar_valid and self.radar_detection:
                self.update_radar()
            
            # Fore Camera
            if self.camera_fore_valid and self.fore_camera_detection:
                self.update_camera(orientation="fore")
            
            # Aft Camera
            if self.camera_aft_valid and self.aft_camera_detection:
                self.update_camera(orientation="aft")
            
            # Port Camera
            if self.camera_port_valid and self.port_camera_detection:
                self.update_camera(orientation="port")
            
            # Starboard Camera
            if self.camera_starboard_valid and self.starboard_camera_detection:
                self.update_camera(orientation="starboard")
                
            # ========================================
            # Observer outputs
            # ========================================
            self.estimated_n    = self.x[0]
            self.estimated_e    = self.x[1]
            self.estimated_vn    = self.x[2]
            self.estimated_ve    = self.x[3]
                
        except Exception as e:
            # IMPORTANT: do not crash host
            print(f"[TargetShipTrackingEKF] ERROR t={current_time} dt={step_size}: {type(e).__name__}: {e}")
            print(traceback.format_exc())
            
            # Freeze dynamics safely (keep last state/outputs)
            self.estimated_n    = self.x[0]
            self.estimated_e    = self.x[1]
            self.estimated_vn    = self.x[2]
            self.estimated_ve    = self.x[3]
        
        return True