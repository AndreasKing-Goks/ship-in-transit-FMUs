import numpy as np
import copy
from typing import NamedTuple

def wrap_pi(a):
    return (a + np.pi) % (2*np.pi) - np.pi

class CurrentModelConfiguration(NamedTuple):
    initial_current_velocity: float
    current_velocity_standard_deviation: float
    current_velocity_decay_rate: float
    initial_current_direction: float
    current_direction_standard_deviation: float
    current_direction_decay_rate: float
    timestep_size: float

class SurfaceCurrent:
    def __init__(self, config:CurrentModelConfiguration, seed=None, clip_speed_nonnegative=True):
        '''
        Small mu value means a slow varying process. While large mu_value meas a fast-decaying fluctuations.
        '''
        # Random seed
        self.rng = np.random.default_rng(seed)  # private RNG
        
        # Config
        self.config = config
        
        # Initialize state variables (velocity magnitude and direction)
        self.vel = config.initial_current_velocity                    # mean current velocity magnitude [m/s]
        self.mu_vel = config.current_velocity_decay_rate              # decay rate for velocity (Gauss–Markov)

        self.dir = config.initial_current_direction                   # initial current direction [rad]
        self.mu_dir = config.current_direction_decay_rate             # decay rate for direction (Gauss–Markov)

        # Standard deviations for the white noise inputs
        self.sigma_vel = config.current_velocity_standard_deviation   # noise strength for velocity
        self.sigma_dir = config.current_direction_standard_deviation  # noise strength for direction

        self.dt = config.timestep_size                                # timestep size [s]
        
        # Clip negative speed
        self.clip_speed_nonnegative= clip_speed_nonnegative
        
        # Record of the initial parameters
        self.record_initial_parameters()
    
    def set_seed(self, seed: int | None):
        # Allow reseeding at any time
        self.rng = np.random.default_rng(seed)
        
    def compute_current_velocity(self, vel_mean):
        # Generate Gaussian white noise for velocity.
        # Scale by 1/sqrt(dt) so variance is consistent with continuous-time noise
        w = self.rng.normal(0, self.sigma_vel / np.sqrt(self.dt))  
        
        # Update velocity using Euler discretization of: Vdot + mu*V = w
        self.vel = self.vel + self.dt * (-self.mu_vel * (self.vel - np.abs(vel_mean)) + w)
        
        return self.vel
    
    def compute_current_direction(self, dir_mean):
        # ensure means live on the same branch
        dir_mean = wrap_pi(dir_mean)
        self.dir = wrap_pi(self.dir)        
        
        # shortest signed angular error in (-pi, pi]
        err = wrap_pi(self.dir - dir_mean)       

        # Generate Gaussian white noise for direction
        w = self.rng.normal(0, self.sigma_dir / np.sqrt(self.dt))  
        
        # Update direction using Euler discretization of: ψdot + mu*ψ = w
        self.dir = wrap_pi(self.dir + self.dt * (-self.mu_dir * err + w))
        
        return self.dir
    
    def get_current_vel_and_dir(self, vel_mean=None, dir_mean=None):
        '''
        vel_mean = action outputed by RL agent. 
        [vel_mean > 0], negative value will automatically reversed to positive
        
        dir_mean = action outputed by RL agent.
        '''
        if vel_mean is None: vel_mean = 0.0
        if dir_mean is None: dir_mean = 0.0
        
        # Update both velocity and direction and return them
        U_c = self.compute_current_velocity(vel_mean)
        if self.clip_speed_nonnegative:
            U_c = max(0.0, U_c)
        psi_c = self.compute_current_direction(dir_mean)
        
        return U_c, psi_c
    
    def record_initial_parameters(self):
        '''
        Internal method to take a record of internal attributes after __init__().
        This record will be used to reset the model later without re-instantiation.
        '''
        self._initial_parameters = {
            key: copy.deepcopy(self.__dict__[key])
            for key in ['vel', 'mu_vel', 'dir', 'mu_dir',
                        'sigma_vel', 'sigma_dir', 'dt',
                        'clip_speed_nonnegative']
            }

    def _reset_params_to_initial(self):
        for key, value in self._initial_parameters.items():
            setattr(self, key, copy.deepcopy(value))
            
    # Gym-style reset that can accept a seed
    def reset(self, seed: int | None = None):
        if seed is not None:
            self.set_seed(seed)
        self._reset_params_to_initial()