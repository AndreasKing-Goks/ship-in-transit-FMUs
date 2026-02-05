"""
Surface Current Model Python FMU implementation.
This FMU manages the surface current model based on Gaussian noise.

Authors : Andreas R.G. Sitorus
Date    : Februari 2026
"""

from pythonfmu import Fmi2Causality, Fmi2Slave, Fmi2Variability, Real, Integer, Boolean, String
import numpy as np

class SurfaceCurrentModel(Fmi2Slave):
    
    author = "Andreas R.G. Sitorus"
    description = "Surface Current Model Python FMU Implementation"
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        '''
        Small mu value means a slow varying process. While large mu_value meas a fast-decaying fluctuations.
        '''
        ## Parameters
        self.seed                                   = 0
        
        self.initial_current_speed                  = 0.0
        self.current_velocity_decay_rate            = 0.0
        self.current_velocity_standard_deviation    = 0.0
        
        self.initial_current_direction              = 0.0
        self.current_direction_decay_rate           = 0.0
        self.current_direction_standard_deviation   = 0.0

        self.clip_speed_nonnegative                 = True
        
        # Input
        self.mean_current_speed                     = 0.0
        self.mean_current_direction                 = 0.0
        
        # Output
        self.current_speed                          = 0.0
        self.current_direction                      = 0.0
        
        # Registration
        # =========================
        # Parameters
        # =========================

        # Random seed
        self.register_variable(Integer("seed", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))

        # Current speed parameters
        self.register_variable(Real("initial_current_speed", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("current_velocity_decay_rate", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("current_velocity_standard_deviation", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))

        # Current direction parameters
        self.register_variable(Real("initial_current_direction", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("current_direction_decay_rate", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("current_direction_standard_deviation", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))

        # Boolean flags
        self.register_variable(Boolean("clip_speed_nonnegative", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))

        # =========================
        # Input Variables
        # =========================
        self.register_variable(Real("mean_current_speed", causality=Fmi2Causality.input))
        self.register_variable(Real("mean_current_direction", causality=Fmi2Causality.input))

        # =========================
        # Output Variables
        # =========================
        self.register_variable(Real("current_speed", causality=Fmi2Causality.output))
        self.register_variable(Real("current_direction", causality=Fmi2Causality.output))

        
    def pre_compute_initial_parameters(self):
        # Random seed
        self.rng = np.random.default_rng(self.seed)  # private RNG

        self.dir = self.initial_current_direction                   # initial current direction [rad]
        self.mu_dir = self.current_direction_decay_rate             # decay rate for direction (Gauss–Markov)

        # Standard deviations for the white noise inputs
        self.sigma_vel = self.current_velocity_standard_deviation   # noise strength for velocity
        self.sigma_dir = self.current_direction_standard_deviation  # noise strength for direction
        
    
    def wrap_pi(self, a):
        return (a + np.pi) % (2*np.pi) - np.pi
    
    
    def set_seed(self, seed: int | None):
        # Allow reseeding at any time
        self.rng = np.random.default_rng(seed)
        
        
    def compute_current_speed(self, mean_current_speed, step_size):
        # Generate Gaussian white noise for velocity.
        # Scale by 1/sqrt(dt) so variance is consistent with continuous-time noise
        w = self.rng.normal(0, self.sigma_vel / np.sqrt(step_size))  
        
        # Update velocity using Euler discretization of: Vdot + mu*V = w
        current_speed = self.initial_current_speed + \
            step_size * (-self.mu_vel * (self.self.initial_current_speed - np.abs(mean_current_speed)) + w)
        
        return current_speed
        
    
    def compute_current_direction(self, mean_current_direction, step_size):
        # ensure means live on the same branch
        mean_current_direction = self.wrap_pi(mean_current_direction)
        current_direction = self.wrap_pi(self.initial_current_direction)        
        
        # shortest signed angular error in (-pi, pi]
        err = self.wrap_pi(current_direction - mean_current_direction)       

        # Generate Gaussian white noise for direction
        w = self.rng.normal(0, self.sigma_dir / np.sqrt(step_size))  
        
        # Update direction using Euler discretization of: ψdot + mu*ψ = w
        current_direction = self.wrap_pi(current_direction + step_size * (-self.mu_dir * err + w))
        
        return current_direction
    
        
    def do_step(self, current_time: float, step_size: float) -> bool:
        '''
        mean_current_speed = action outputed by RL agent. 
        [mean_current_speed > 0], negative value will automatically reversed to positive
        
        mean_current_direction = action outputed by RL agent.
        '''
        if self.mean_current_speed is None: self.mean_current_speed = 0.0
        if self.mean_current_direction is None: self.mean_current_direction = 0.0
        
        # Pre-compute the initial parameters
        self.pre_compute_initial_parameters()
        
        # Update both velocity and direction and return them
        current_speed     = self.compute_current_speed(self.mean_current_speed, step_size)
        if self.clip_speed_nonnegative:
            current_speed = max(0.0, current_speed)
        current_direction = self.compute_current_direction(self.mean_current_direction, step_size)
        
        # Assign the results to the output variables
        self.current_speed      = current_speed
        self.current_direction  = current_direction
        
        # Assign the results to the memory variables
        self.initial_current_speed      = current_speed
        self.initial_current_direction  = current_direction
        
        return True