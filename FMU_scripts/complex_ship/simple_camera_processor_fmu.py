"""
Simple Camera Processor Python FMU implementation.

Authors : Andreas R.G. Sitorus
Date    : September 2026
"""

from pythonfmu import Fmi2Causality, Fmi2Slave, Fmi2Variability, Real, Integer, Boolean, String
import numpy as np
import traceback

class CameraProcessor(Fmi2Slave):
    author = "Andreas R.G. Sitorus"
    description = "Simple Camera Processor Python FMU Implementation"
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        
        # Internal Variables
        self._precomputed                                   = False
        self._camera_positions                              = ["fore", "aft", "port", "starboard"]
        
        ## Noise Parameters (Sigma = Standard Deviation)
        self.max_target_ship_count                          = 4         # Up to four
        self.max_camera_count                               = 4         # Up to four
        self.length_of_ship                                 = 80
        self.width_of_ship                                  = 16
        self.initial_own_north                              = 0.0
        self.initial_own_east                               = 0.0
        self.initial_own_yaw_angle                          = 0.0
        for cam_pos in self._camera_positions:
            # For each camera
            for i in range(1,4):
                # For each ship, output the target ship distance and bearing to camera
                setattr(self, f"init_{cam_pos}_detect_tar_{i}", False)
                setattr(self, f"init_{cam_pos}_tar_{i}_distance_to_camera", 1e9)
                setattr(self, f"init_{cam_pos}_tar_{i}_bearing_to_camera", 0.0)
        
        ## Input
        self.own_north                                      = 0.0
        self.own_east                                       = 0.0
        self.own_yaw_angle                                  = 0.0       # radian
        for cam_pos in self._camera_positions:
            # For each camera
            for i in range(1,4):
                # For each ship, output the target ship distance and bearing to camera
                setattr(self, f"{cam_pos}_detect_tar_{i}", False)
                setattr(self, f"{cam_pos}_tar_{i}_distance_to_camera", 1e9)
                setattr(self, f"{cam_pos}_tar_{i}_bearing_to_camera", 0.0)
        
        ## Output
        for i in range(1,4):
            # For each ship, set the initial position to be far away, facing north, and static
            setattr(self, f"tar_{i}_north", 1e9)
            setattr(self, f"tar_{i}_east", 1e9)
        
        ## Registration
        # PI Parameters
        self.register_variable(Integer("max_target_ship_count", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Integer("max_camera_count", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("length_of_ship", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
        self.register_variable(Real("width_of_ship", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
        self.register_variable(Real("initial_own_north", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
        self.register_variable(Real("initial_own_east", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
        self.register_variable(Real("initial_own_yaw_angle", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
        for cam_pos in self._camera_positions:
            # For each camera
            for i in range(1,4):
                self.register_variable(Boolean(f"init_{cam_pos}_detect_tar_{i}", causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))
                self.register_variable(Real(f"init_{cam_pos}_tar_{i}_distance_to_camera", causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))
                self.register_variable(Real(f"init_{cam_pos}_tar_{i}_bearing_to_camera", causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))
        
        # Input
        self.register_variable(Real("own_north", causality=Fmi2Causality.input))
        self.register_variable(Real("own_east", causality=Fmi2Causality.input))
        self.register_variable(Real("own_yaw_angle", causality=Fmi2Causality.input))
        for cam_pos in self._camera_positions:
            # For each camera
            for i in range(1,4):
                self.register_variable(Boolean(f"{cam_pos}_detect_tar_{i}", causality=Fmi2Causality.input, variability=Fmi2Variability.discrete))
                self.register_variable(Real(f"{cam_pos}tar_{i}_distance_to_camera", causality=Fmi2Causality.input))
                self.register_variable(Real(f"{cam_pos}tar_{i}_bearing_to_camera", causality=Fmi2Causality.input))

        # Output
        for i in range(1,4):
            self.register_variable(Real(f"tar_{i}_north", causality=Fmi2Causality.output))
            self.register_variable(Real(f"tar_{i}_east", causality=Fmi2Causality.output))
        
    
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
                self.own_north                  = self.initial_own_north
                self.own_east                   = self.initial_own_east
                self.own_yaw_angle              = self.initial_own_yaw_angle
                
                # Set the initial camera informations                    
                for cam_pos in self._camera_positions:
                    # For each camera
                    for i in range(1,4):
                        # For each ship, output the target ship distance and bearing to camera
                        setattr(self, f"{cam_pos}_detect_tar_{i}", getattr(self, f"init_{cam_pos}_detect_tar_{i}"))
                        setattr(self, f"{cam_pos}_tar_{i}_distance_to_camera", getattr(self, f"init_{cam_pos}_tar_{i}_north"))
                        setattr(self, f"{cam_pos}_tar_{i}_bearing_to_camera", getattr(self, f"init_{cam_pos}_tar_{i}_north"))
                
                self._precomputed               = True
            
            # Compute the amount of target ships
            c = int(self.max_target_ship_count)
            c = max(0, min(c,3))
            
            # Container
            distance_to_camera_dict = {f"tar_{i}":[] for i in range(1, c+1)}
            bearing_to_camera_dict = {f"tar_{i}":[] for i in range(1, c+1)}
            
            # Check if camera detect something
            for cam_pos in self._camera_positions:
                # For each camera
                    for i in range(1,4):
                        detect = getattr(self, f"{cam_pos}_detect_tar_{i}")
                        
                        if detect:
                            distance_to_camera  = getattr(self, f"{cam_pos}_tar_{i}_distance_to_camera")
                            bearing_to_camera   = getattr(self, f"{cam_pos}_tar_{i}_bearing_to_camera")
                            
                            distance_to_camera_dict[f"tar_{i}"].append(distance_to_camera)
                            bearing_to_camera_dict[f"tar_{i}"].append(bearing_to_camera)
            
            # Process data
            for i in range(1,c+1):
                if distance_to_camera_dict[f"tar_{i}"] is not None:
                    mean_distance_to_camera = np.mean(distance_to_camera_dict[f"tar_{i}"])
                else:
                    mean_distance_to_camera = None
                
                if bearing_to_camera_dict[f"tar_{i}"] is not None:
                    mean_bearing_to_camera  = np.mean(bearing_to_camera_dict[f"tar_{i}"])
                else:
                    mean_bearing_to_camera  = None
                
                if mean_distance_to_camera is None and mean_bearing_to_camera is None:
                    setattr(self, f"tar_{i}_north", 1e9)
                    setattr(self, f"tar_{i}_east", 1e9)
                    continue
                
                
            
            self.measurement_valid  = True
        
        except Exception as e:
            # IMPORTANT: do not crash host
            print(f"[CameraProcessor] ERROR t={current_time} dt={step_size}: {type(e).__name__}: {e}")
            print(traceback.format_exc())
            
            # Freeze dynamics safely (keep last state/outputs)
            self.measurement_valid                  = False
        
            return True