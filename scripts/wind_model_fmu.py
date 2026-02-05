"""
Wind Model Python FMU implementation.
This FMU manages the wind model based on Gaussian noise.

Authors : Andreas R.G. Sitorus
Date    : Februari 2026
"""

from pythonfmu import Fmi2Causality, Fmi2Slave, Fmi2Variability, Real, Integer, Boolean, String
import numpy as np
import copy

class WindModel(Fmi2Slave):
    
    author = "Andreas R.G. Sitorus"
    description = "Wind Model Python FMU Implementation"
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        ## Parameters
        self.seed   = 0
        
        self.initial_mean_wind_speed                    = 0.0
        self.mean_wind_speed_decay_rate                 = 0.0
        self.mean_wind_speed_standard_deviation         = 0.0
        
        self.initial_wind_direction                     = 0.0
        self.wind_direction_decay_rate                  = 0.0
        self.wind_direction_standard_deviation          = 0.0
        
        self.minimum_mean_wind_speed                    = 0.0
        self.maximum_mean_wind_speed                    = 0.0
        self.minimum_wind_gust_frequency                = 0.0
        self.maximum_wind_gust_frequency                = 0.0
        self.wind_gust_frequency_discrete_unit_count    = 0
        
        self.wind_evaluation_height                     = 0.0
        self.U10                                        = 0.0
        self.kappa_parameter                            = 0.0
        
        self.clip_speed_nonnegative                     = True
        
        # Input
        self.mean_wind_speed                            = 0.0
        self.mean_wind_direction                        = 0.0
        
        # Output
        self.wind_speed                                 = 0.0
        self.wind_direction                             = 0.0
        
        # Compute spectrum flag (only compute spectrum once)
        self.spectrum_is_computed                       = False
        
        # Registration
        # =========================
        # Parameters
        # =========================

        # Random seed
        self.register_variable(Integer("seed", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))

        # Mean wind speed parameters
        self.register_variable(Real("initial_mean_wind_speed", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("mean_wind_speed_decay_rate", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("mean_wind_speed_standard_deviation", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))

        # Wind direction parameters
        self.register_variable(Real("initial_wind_direction", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("wind_direction_decay_rate", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("wind_direction_standard_deviation", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))

        # Wind constraints and gust parameters
        self.register_variable(Real("minimum_mean_wind_speed", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("maximum_mean_wind_speed", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("minimum_wind_gust_frequency", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("maximum_wind_gust_frequency", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Integer("wind_gust_frequency_discrete_unit_count", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))

        # Wind profile / log-law parameters
        self.register_variable(Real("wind_evaluation_height", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("U10", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("kappa_parameter", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))

        # Boolean flags
        self.register_variable(Boolean("clip_speed_nonnegative", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))

        # =========================
        # Input Variables
        # =========================
        self.register_variable(Real("mean_wind_speed", causality=Fmi2Causality.input))
        self.register_variable(Real("mean_wind_direction", causality=Fmi2Causality.input))

        # =========================
        # Output Variables
        # =========================
        self.register_variable(Real("wind_speed", causality=Fmi2Causality.output))
        self.register_variable(Real("wind_direction", causality=Fmi2Causality.output))


    # ----- spectra -----
    def _norsok(self, f):
        # S(f) = 320 (U10/10)^2 (z/10)^0.45 / (1 + x^n)^(5/(3n))
        # x = 172 f (z/10)^(2/3) (U10/10)^(-3/4), n = 0.468
        n = 0.468
        x = 172.0 * f * (self.z/10.0)**(2.0/3.0) * (self.U10/10.0)**(-3.0/4.0)
        return 320.0 * (self.U10/10.0)**2 * (self.z/10.0)**0.45 / (1.0 + x**n)**(5.0/(3.0*n))
    
        
    def pre_compute_initial_parameters(self):
        # Random seed
        self.rng = np.random.default_rng(self.seed)  # private RNG

        # frequency grid
        self.f_min = self.minimum_wind_gust_frequency
        self.f_max = self.maximum_wind_gust_frequency
        self.N_f = self.wind_gust_frequency_discrete_unit_count
        self.f = np.linspace(self.f_min, self.f_max, self.N_f)   # Hz
        self.df = self.f[1] - self.f[0]
        
        if self.spectrum_is_computed:
            # ---- precompute gust amplitudes & phases (fixed), keep running angles ----
            S = self._norsok(self.f)
            self.a = np.sqrt(2.0 * S * self.df)                  # (N_f,)
            self.phi0 = 2*np.pi*self.rng.random(self.N_f)        # fixed phases
            self.theta = self.phi0.copy()                        # running angles
            
            self.spectrum_is_computed = True
        
    
    def wrap_pi(self, a):
        return (a + np.pi) % (2*np.pi) - np.pi
    
    
    def set_seed(self, seed: int | None):
        # Allow reseeding at any time
        self.rng = np.random.default_rng(seed)
        

    def compute_wind_mean_speed(self, Ubar_mean, step_size):
        # Ȗ_{k+1} = Ȗ_k + dt(-μ Ȗ_k + w), w ~ N(0, σ^2/dt)
        w = self.rng.normal(0.0, self.mean_wind_speed_standard_deviation/np.sqrt(step_size))
        Ubar = self.initial_mean_wind_speed + step_size * (-self.mean_wind_speed_decay_rate * (self.initial_mean_wind_speed - Ubar_mean) + w)
        Ubar = np.clip(Ubar, self.minimum_mean_wind_speed, self.maximum_mean_wind_speed)
        return Ubar


    def compute_wind_gust(self, step_size):
        # advance phases by 2π f dt and sum components
        self.theta += 2*np.pi*self.f*step_size
        return np.sum(self.a * np.cos(self.theta))


    def compute_wind_direction(self, dir_mean, step_size):
        # ensure means live on the same branch
        dir_mean = self.wrap_pi(dir_mean)
        dir = self.wrap_pi(self.initial_wind_direction) 
        
        # shortest signed angular error in (-pi, pi]
        err = self.wrap_pi(dir - dir_mean)   
        
        # Generate Gaussian white noise for direction
        w = self.rng.normal(0.0, self.wind_direction_standard_deviation/np.sqrt(step_size))
        
        # Update direction using Euler discretization of: ψdot + mu*ψ = w
        dir = self.wrap_pi(dir + step_size * (-self.wind_direction_decay_rate * (err) + w))
        
        return dir
    
        
    def do_step(self, current_time: float, step_size: float) -> bool:
        '''
        vel_mean = action outputed by RL agent. 
        [vel_mean > 0], negative value will automatically reversed to positive
        
        dir_mean = action outputed by RL agent.
        '''
        # Pre-compute the initial parameters
        self.pre_compute_initial_parameters()
        
        Ubar = self.compute_wind_mean_velocity(self.mean_wind_speed, step_size)
        Ug   = self.compute_wind_gust(step_size)
        wind_speed    = Ubar + Ug
        if self.clip_speed_nonnegative:
            wind_speed = max(0.0, wind_speed)
        wind_direction  = self.compute_wind_direction(self.mean_wind_direction, step_size)
        
        # Assign the results to the output variables
        self.wind_speed      = wind_speed
        self.wind_direction  = wind_direction
        
        # Assign the results to the memory variables
        self.initial_mean_wind_speed = Ubar
        self.initial_wind_direction  = wind_direction
        
        return True