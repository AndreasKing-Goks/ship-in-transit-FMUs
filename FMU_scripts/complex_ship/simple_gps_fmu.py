"""
Simple GPS Sensor Python FMU implementation.

Authors : Andreas R.G. Sitorus
Date    : September 2026
"""

from pythonfmu import Fmi2Causality, Fmi2Slave, Fmi2Variability, Real, Integer, Boolean, String
import numpy as np
import traceback

class GPSSensor(Fmi2Slave):
    author = "Andreas R.G. Sitorus"
    description = "GPS Sensor Python FMU Implementation"
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        
        ## Noise Parameters (Sigma = Standard Deviation)
        self.seed                           = 42
        self.north_sigma                    = 2.0
        self.east_sigma                     = 2.0
        
        ## Input
        self.freeze                         = False
        self.north                          = 0.0
        self.east                           = 0.0
        
        ## Output
        self.measured_north                 = 0.0
        self.measured_east                  = 0.0
        self.measurement_valid              = True
        
        # Internal Variables
        self._noise_generator               = None
        self._prev_measured_north           = 0.0
        self._prev_measured_east            = 0.0
        
        ## Registration
        # PI Parameters
        self.register_variable(Integer("seed", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        self.register_variable(Real("north_sigma", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
        self.register_variable(Real("east_sigma", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
        
        # Input
        self.register_variable(Boolean("freeze", causality=Fmi2Causality.input, variability=Fmi2Variability.discrete))
        self.register_variable(Real("north", causality=Fmi2Causality.input))
        self.register_variable(Real("east", causality=Fmi2Causality.input))

        # Output
        self.register_variable(Real("measured_north", causality=Fmi2Causality.output))
        self.register_variable(Real("measured_east", causality=Fmi2Causality.output))
        self.register_variable(Boolean("measurement_valid", causality=Fmi2Causality.output, variability=Fmi2Variability.discrete))
        
    def do_step(self, current_time: float, step_size: float) -> bool:
        try:
            # Precompute once
            if self._noise_generator is None:
                self._noise_generator           = np.random.default_rng(seed=self.seed)
            
            if not self.freeze:
                # Noise only generated when the sensor is not frozen
                north_noise                     = self._noise_generator.normal(loc=0.0, scale=self.north_sigma)
                east_noise                      = self._noise_generator.normal(loc=0.0, scale=self.east_sigma)
                
                self.measured_north             = self.north + north_noise 
                self.measured_east              = self.east + east_noise 
            # Sensor stuck
            else:
                self.measured_north             = self._prev_measured_north
                self.measured_east              = self._prev_measured_east 
            
            self._prev_measured_north           = self.measured_north
            self._prev_measured_east            = self.measured_east
            self.measurement_valid              = True
        
        except Exception as e:
            # IMPORTANT: do not crash host
            print(f"[GPSSensor] ERROR t={current_time} dt={step_size}: {type(e).__name__}: {e}")
            print(traceback.format_exc())
            
            # Freeze dynamics safely (keep last state/outputs)
            self.measurement_valid              = False
        
        return True