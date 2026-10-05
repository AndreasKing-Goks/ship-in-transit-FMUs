"""
Camera Sensor Python FMU implementation.

Authors : Andreas R.G. Sitorus
Date    : September 2026
"""

from pythonfmu import Fmi2Causality, Fmi2Slave, Fmi2Variability, Real, Integer, Boolean, String
import numpy as np
import traceback

class CameraSensor(Fmi2Slave):
    author = "Andreas R.G. Sitorus"
    description = "Camera Sensor Python FMU Implementation"
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        
        ## Noise Parameters (Sigma = Standard Deviation)
        self.seed                                           = 42
        self.max_target_ship_count                          = 4         # Up to four
        self.length_of_ship                                 = 80
        self.width_of_ship                                  = 16
        self.ship_distance_sigma                            = 5.0
        self.low_visibility_ship_distance_base_noise        = 500.0
        self.low_visibility_ship_distance_sigma             = 250.0
        self.bearing_to_camera_sigma_deg                    = 0.1       # deg
        self.low_visibility_bearing_to_camera_sigma_deg     = 1.0       # deg
        self.initial_ship_distance                          = 0.0
        self.initial_bearing_to_camera                      = 0.0
        self.initial_own_north                              = 0.0
        self.initial_own_east                               = 0.0
        self.initial_own_yaw_angle                          = 0.0
        self.camera_position                                = "fore"    # 4 position: "fore", "aft", "port", "starboard"
        self.camera_far_depth_of_field_limit                = 2000
        self.camera_horizontal_field_of_view_angle_limit    = 60        # deg
        for i in range(1,4):
            # For each ship, set the initial position to be far away, facing north, and static
            setattr(self, f"init_tar_{i}_north", 1e9)
            setattr(self, f"init_tar_{i}_east", 1e9)
        
        ## Input
        self.own_north                                      = 0.0
        self.own_east                                       = 0.0
        self.own_yaw_angle                                  = 0.0       # radian
        for i in range(1,4):
            # For each ship, set the initial position to be far away, facing north, and static
            setattr(self, f"tar_{i}_north", 1e9)
            setattr(self, f"tar_{i}_east", 1e9)
        self.low_visibility                                 = False
        
        ## Output
        self.measurement_valid                              = False
        self.camera_position_out                            = "fore"    # 4 position: "fore", "aft", "port", "starboard"
        for i in range(1,4):
            # For each ship, output the target ship distance and bearing to camera
            setattr(self, f"detect_tar_{i}", False)
            setattr(self, f"tar_{i}_distance_to_camera", 1e9)
            setattr(self, f"tar_{i}_bearing_to_camera", 0.0)
        
        # Internal Variables
        self._precomputed                                   = False
        
        ## Registration
        # PI Parameters
        self.register_variable(Integer("seed", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        self.register_variable(Integer("max_target_ship_count", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("length_of_ship", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
        self.register_variable(Real("width_of_ship", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
        self.register_variable(Real("ship_distance_sigma", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
        self.register_variable(Real("low_visibility_ship_distance_base_noise", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
        self.register_variable(Real("low_visibility_ship_distance_sigma", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
        self.register_variable(Real("bearing_to_camera_sigma_deg", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
        self.register_variable(Real("low_visibility_bearing_to_camera_sigma_deg", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
        self.register_variable(Real("initial_ship_distance", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
        self.register_variable(Real("initial_bearing_to_camera", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
        self.register_variable(Real("initial_own_north", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
        self.register_variable(Real("initial_own_east", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
        self.register_variable(Real("initial_own_yaw_angle", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
        self.register_variable(String("camera_position", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        self.register_variable(Real("camera_far_depth_of_field_limit", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
        self.register_variable(Real("camera_horizontal_field_of_view_angle_limit", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
        for i in range(1,4):
            self.register_variable(Real(f"init_tar_{i}_north", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
            self.register_variable(Real(f"init_tar_{i}_east", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
        
        # Input
        self.register_variable(Real("own_north", causality=Fmi2Causality.input))
        self.register_variable(Real("own_east", causality=Fmi2Causality.input))
        self.register_variable(Real("own_yaw_angle", causality=Fmi2Causality.input))
        for i in range(1,4):
            self.register_variable(Real(f"tar_{i}_north", causality=Fmi2Causality.input))
            self.register_variable(Real(f"tar_{i}_east", causality=Fmi2Causality.input))
        self.register_variable(Boolean("low_visibility", causality=Fmi2Causality.input, variability=Fmi2Variability.discrete))

        # Output
        self.register_variable(Boolean("measurement_valid", causality=Fmi2Causality.output, variability=Fmi2Variability.discrete))
        self.register_variable(String("camera_position_out", causality=Fmi2Causality.output))
        for i in range(1,4):
            self.register_variable(Boolean(f"detect_tar_{i}", causality=Fmi2Causality.output, variability=Fmi2Variability.discrete))
            self.register_variable(Real(f"tar_{i}_distance_to_camera", causality=Fmi2Causality.output))
            self.register_variable(Real(f"tar_{i}_bearing_to_camera", causality=Fmi2Causality.output))
    
    def _wrap_to_pi(self, a):
        return (a + np.pi) % (2*np.pi) - np.pi
        
    def _wrap_to_half_pi(self, a):
        return (a + np.pi/2) % np.pi - np.pi/2
        
    def do_step(self, current_time: float, step_size: float) -> bool:
        try:
            # Precompute once
            if not self._precomputed:
                self._noise_generator           = np.random.default_rng(seed=self.seed)
                
                # Set the own ship initial states
                self.own_north      = self.initial_own_north
                self.own_east       = self.initial_own_east
                self.own_yaw_angle  = self.initial_own_yaw_angle
                
                # Set the target ships initial states
                for i in range(1,4):
                    # For each ship, set the initial position to be far away, facing north, and static
                    setattr(self, f"tar_{i}_north", getattr(self, f"init_tar_{i}_north"))
                    setattr(self, f"tar_{i}_east", getattr(self, f"init_tar_{i}_east"))
                    
                # Base facing angle for camera
                if self.camera_position == "fore":
                    self.theta_cam  = 0.0
                elif self.camera_position == "aft":
                    self.theta_cam  = np.pi
                elif self.camera_position == "starboard":
                    self.theta_cam  = np.pi/2
                elif self.camera_position == "port":
                    self.theta_cam  = -np.pi/2
            
                self._precomputed               = True
                
            # Compute the amount of target ships
            c = int(self.max_target_ship_count)
            c = max(0, min(c,3))
            
            for i in range (1, c+1):
                north = getattr(self, f"tar_{i}_north")
                east  = getattr(self, f"tar_{i}_east")
                yaw   = getattr(self, f"tar_{i}_yaw_angle")
                yaw   = self._wrap_to_pi(yaw)
                
                ### INITIATE
                p_own                  = np.array([self.own_north, self.own_east], dtype=float)
                p_tar_list             = []
                
                p_tar = np.array([north, east], dtype=float)
                p_tar_list.append(p_tar)
            
            
            for i in range (1, c):
                ## Distance
                # Relative distance
                r = p_tar_list[i] - p_own
                
                # Target ship to own ship distance
                dist = np.hypot(r[1], r[0])    

                ## Relative Bearing
                theta       = np.arctan2(r[1], r[0])                              # theta -> North to target ship
                beta        = self._wrap_to_pi(theta - self.own_yaw_angle)        # psi -> North to own heading; beta -> own heading to target ship
                
                # x_distance and y_distance (w.r.t. camera)
                x_distance  = dist * np.sin(beta)
                y_distance  = dist * np.cos(beta)
                
                # distance to camera, depends on position
                if self.camera_position == "fore" or self.camera_position == "aft":
                    x_dist_cam  = x_distance
                    y_dist_cam  = y_distance - self.length_of_ship/2
                elif self.camera_position == "port" or self.camera_position == "starboard":
                    x_dist_cam  = x_distance - self.width_of_ship/2
                    y_dist_cam  = y_distance
                
                # Pre-noised distance and bearing to camera    
                dist_to_cam     = np.hypot(y_dist_cam, x_dist_cam)
                bearing_to_cam  = np.arctan2(y_dist_cam, x_dist_cam)
                
                if not self.low_visibility:
                    # Bearing angle noise
                    bearing_to_cam_noise = self._noise_generator.normal(loc=0.0, scale=self.bearing_to_camera_sigma)
                    
                    # Distance length noise
                    ship_distance_noise     = self._noise_generator.normal(loc=0.0, scale=self.ship_distance_sigma)
                    
                # Sensor stuck
                else:
                    # Bearing angle noise
                    bearing_to_cam_noise = self._noise_generator.normal(loc=0.0, scale=self.low_visibility_bearing_to_camera_sigma)
                    
                    # Distance length noise
                    ship_distance_noise     = self.low_visibility_ship_distance_base_noise + self._noise_generator.normal(loc=0.0, scale=self.low_visibility_ship_distance_sigma)
                
                ## Measured output            
                measured_dist_to_cam    = dist_to_cam + ship_distance_noise
                measured_bearing_to_cam = bearing_to_cam + bearing_to_cam_noise
                
                # Check if the ship is detected (within the cone?)
                ship_detected   = (measured_dist_to_cam < self.camera_far_depth_of_field_limit) and (measured_bearing_to_cam < self.bearing_to_camera_sigma_deg)
                
                if ship_detected:
                    setattr(self, f"detect_tar_{i}", True)
                    setattr(self, f"tar_{i}_distance_to_camera", measured_dist_to_cam)
                    setattr(self, f"tar_{i}_bearing_to_camera", measured_bearing_to_cam)
                else:
                    setattr(self, f"detect_tar_{i}", False)
                    setattr(self, f"tar_{i}_distance_to_camera", 1e9)
                    setattr(self, f"tar_{i}_bearing_to_camera", 0.0)
            
            self.measurement_valid  = True
        
        except Exception as e:
            # IMPORTANT: do not crash host
            print(f"[CameraSensor] ERROR t={current_time} dt={step_size}: {type(e).__name__}: {e}")
            print(traceback.format_exc())
            
            # Freeze dynamics safely (keep last state/outputs)
            self.measurement_valid                  = False
        
            return True