"""
Simple Shaft Speed Log Sensor Python FMU implementation.

Authors : Andreas R.G. Sitorus
Date    : September 2026
"""

from pythonfmu import Fmi2Causality, Fmi2Slave, Fmi2Variability, Real, Integer, Boolean, String
import numpy as np
import traceback

class ShaftSpeedLogSensor(Fmi2Slave):
    author = "Andreas R.G. Sitorus"
    description = "Shaft Speed Log Sensor Python FMU Implementation"
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        
        ## Noise Parameters (Sigma = Standard Deviation)
        self.seed                               = 42
        self.shaft_speed_rpm_sigma              = 0.5       # [rpm]
        
        ## Input
        self.freeze                             = False
        self.shaft_speed_rpm                    = 0.0
        
        ## Output
        self.measured_shaft_speed_rpm           = 0.0
        self.measurement_valid                  = True
        
        # Internal Variables
        self._noise_generator                   = None
        self._prev_measured_shaft_speed_rpm     = 0.0
        
        ## Registration
        # PI Parameters
        self.register_variable(Integer("seed", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        self.register_variable(Real("shaft_speed_rpm_sigma", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
        
        # Input
        self.register_variable(Boolean("freeze", causality=Fmi2Causality.input))
        self.register_variable(Real("shaft_speed_rpm", causality=Fmi2Causality.input))

        # Output
        self.register_variable(Real("measured_shaft_speed_rpm", causality=Fmi2Causality.output))
        self.register_variable(Boolean("measurement_valid", causality=Fmi2Causality.output))
        
    def do_step(self, current_time: float, step_size: float) -> bool:
        try:
            # Precompute once
            if self._noise_generator is None:
                self._noise_generator               = np.random.default_rng(seed=self.seed)
            
            if not self.freeze:
                # Noise only generated when the sensor is not frozen
                shaft_speed_rpm_noise               = self._noise_generator.normal(loc=0.0, scale=self.shaft_speed_rpm_sigma)
                
                self.measured_shaft_speed_rpm       = self.shaft_speed_rpm + shaft_speed_rpm_noise
            # Sensor stuck
            else:
                self.measured_shaft_speed_rpm       = self._prev_measured_shaft_speed_rpm
            
            self._prev_measured_shaft_speed_rpm     = self.measured_shaft_speed_rpm
            self.measurement_valid                  = True
        
        except Exception as e:
            # IMPORTANT: do not crash host
            print(f"[ShaftSpeedLogSensor] ERROR t={current_time} dt={step_size}: {type(e).__name__}: {e}")
            print(traceback.format_exc())
            
            # Freeze dynamics safely (keep last state/outputs)
            self.measurement_valid                  = False
        
        return True