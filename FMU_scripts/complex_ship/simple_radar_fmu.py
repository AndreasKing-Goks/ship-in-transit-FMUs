"""
RADAR Python FMU implementation.

Authors : Andreas R.G. Sitorus
Date    : September 2026
"""

from pythonfmu import Fmi2Causality, Fmi2Slave, Fmi2Variability, Real, Integer, Boolean, String
import numpy as np
import traceback

class RADAR(Fmi2Slave):
    author = "Andreas R.G. Sitorus"
    description = "RADAR Python FMU Implementation"
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        
        ## Noise Parameters (Sigma = Standard Deviation)
        self.seed                                           = 42        # Made sure a different seed set for different camera
        self.max_target_ship_count                          = 3         # Up to three
        self.range_sigma                                    = 10.0
        self.bearing_sigma_deg                              = 1.0       # deg
        self.initial_range                                  = 0.0
        self.initial_bearing                                = 0.0
        self.initial_own_north                              = 0.0
        self.initial_own_east                               = 0.0
        self.initial_own_yaw_angle                          = 0.0
        self.max_range                                      = 45_000    # 45 km
        self.radar_degradation_detection_probability_coeff  = 0.5
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
        self.radar_degradation                                 = False
        
        ## Output
        self.measurement_valid                              = False
        for i in range(1,4):
            # For each ship, output the target ship distance and bearing to camera
            setattr(self, f"detect_tar_{i}", False)
            setattr(self, f"tar_{i}_range", 1e9)
            setattr(self, f"tar_{i}_bearing", 0.0)
        
        # Internal Variables
        self._precomputed                                   = False
        
        ## Registration
        # PI Parameters
        self.register_variable(Integer("seed", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        self.register_variable(Integer("max_target_ship_count", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("range_sigma", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
        self.register_variable(Real("bearing_sigma_deg", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
        self.register_variable(Real("initial_range", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
        self.register_variable(Real("initial_bearing", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
        self.register_variable(Real("initial_own_north", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
        self.register_variable(Real("initial_own_east", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
        self.register_variable(Real("initial_own_yaw_angle", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
        self.register_variable(Real("max_range", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
        self.register_variable(Real("radar_degradation_detection_probability_coeff", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
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
        self.register_variable(Boolean("radar_degradation", causality=Fmi2Causality.input, variability=Fmi2Variability.discrete))

        # Output
        self.register_variable(Boolean("measurement_valid", causality=Fmi2Causality.output, variability=Fmi2Variability.discrete))
        for i in range(1,4):
            self.register_variable(Boolean(f"detect_tar_{i}", causality=Fmi2Causality.output, variability=Fmi2Variability.discrete))
            self.register_variable(Real(f"tar_{i}_range", causality=Fmi2Causality.output))
            self.register_variable(Real(f"tar_{i}_bearing", causality=Fmi2Causality.output))
    
    def _wrap_to_pi(self, a):
        return (a + np.pi) % (2*np.pi) - np.pi
    
    def _radar_detection_probability(self, range_):
        """
            Detection probabililty will decay as range increased (Sigmoid)
            Half detection probability at 35000 m
        """
        r50 = 35_000.0
        k = 0.00025

        return 1.0 / (
            1.0 + np.exp(k * (range_ - r50))
        )
        
    def do_step(self, current_time: float, step_size: float) -> bool:
        try:
            # Precompute once
            if not self._precomputed:
                # Noise generator
                self._noise_generator           = np.random.default_rng(seed=self.seed)
                
                # Detection rng
                self._detection_rng             = np.random.default_rng(seed=self.seed)
                
                # Set the own ship initial states
                self.own_north      = self.initial_own_north
                self.own_east       = self.initial_own_east
                self.own_yaw_angle  = self.initial_own_yaw_angle
                
                # Set the target ships initial states
                for i in range(1,4):
                    # For each ship, set the initial position to be far away, facing north, and static
                    setattr(self, f"tar_{i}_north", getattr(self, f"init_tar_{i}_north"))
                    setattr(self, f"tar_{i}_east", getattr(self, f"init_tar_{i}_east"))
            
                self._precomputed               = True
                
            # Compute the amount of target ships
            c = int(self.max_target_ship_count)
            c = max(0, min(c,3))
            
            ### INITIATE
            p_own                  = np.array([self.own_north, self.own_east], dtype=float)
            
            for i in range (c):
                idx = i + 1
                
                ## Distance (# Ship index start at 1)
                north = getattr(self, f"tar_{idx}_north")
                east  = getattr(self, f"tar_{idx}_east")
                
                p_tar = np.array([north, east], dtype=float)
                
                # Relative distance
                r = p_tar - p_own
                
                # Relative target positions in own ship BODY FRAME
                # Transform from BODY to NED (R = [[c -s] [s c]])
                # Transform from NED to BODY (Rinv = [[c s] [-s c]])
                R_inv = np.array([
                    [np.cos(self.own_yaw_angle), np.sin(self.own_yaw_angle)],
                    [-np.sin(self.own_yaw_angle), np.cos(self.own_yaw_angle)]
                ])
                # r[0] -> r[1] -> x: front/aft, y: left/right
                r_body  = R_inv @ r
                x_body  =  r_body[0]    # front/aft
                y_body  =  r_body[1]    # left/right
                
                ## Pre-noised range, range_rate, and bearing to own ship ship (true_range, true_range_rate and true_bearing)   
                # Range
                true_range          = np.hypot(x_body, y_body)
                
                # Bearing
                bearing_body        = np.arctan2(y_body, x_body) # (angle from measured from heading=0)
                true_bearing        = self._wrap_to_pi(bearing_body)
                
                ## Check if the ship is detected 
                # Inside radar range
                within_range        = true_range < self.max_range
                
                ship_visible        = within_range
                detect_rng          = self._detection_rng.uniform(low=0.0, high=1.0)
                
                detection_prob      = self._radar_detection_probability(range=true_range)
                
                if self.radar_degradation:
                    detection_prob      = detection_prob * self.radar_degradation_detection_probability_coeff
                    
                detection = True if detect_rng < detection_prob else False
                
                ## OUTPUT
                # Bearing angle noise
                bearing_noise       = self._noise_generator.normal(loc=0.0, scale=np.deg2rad(self.bearing_sigma_deg))
                    
                # Range noise
                range_noise         = self._noise_generator.normal(loc=0.0, scale=self.range_sigma)
                
                ## Measured output            
                measured_range      = true_range + range_noise
                measured_bearing    = self._wrap_to_pi(true_bearing + bearing_noise)
                detected            = ship_visible and detection
                
                if detected:
                    setattr(self, f"detect_tar_{idx}", True)
                    setattr(self, f"tar_{idx}_range", measured_range)
                    setattr(self, f"tar_{idx}_bearing", measured_bearing)
                else:
                    setattr(self, f"detect_tar_{idx}", False)
                    setattr(self, f"tar_{idx}_range", 1e9)
                    setattr(self, f"tar_{idx}_bearing", 0.0)
            
            self.measurement_valid  = True
        
        except Exception as e:
            # IMPORTANT: do not crash host
            print(f"[RADAR] ERROR t={current_time} dt={step_size}: {type(e).__name__}: {e}")
            print(traceback.format_exc())
            
            # Freeze dynamics safely (keep last state/outputs)
            self.measurement_valid                  = False
        
            return True