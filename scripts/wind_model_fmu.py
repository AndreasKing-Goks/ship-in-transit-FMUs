"""
Wind Model Python FMU implementation.
Mean-driven Ornstein–Uhlenbeck (OU) / Gauss–Markov process for wind.

- RL (or external controller) drives the *mean* wind speed and direction.
- The FMU outputs a stochastic wind that reverts toward those means with inertia.

Authors : Andreas R.G. Sitorus
Date    : February 2026
"""

from pythonfmu import Fmi2Causality, Fmi2Slave, Fmi2Variability, Real, Integer, Boolean, String
import numpy as np
import traceback

class WindModel(Fmi2Slave):
    
    author = "Andreas R.G. Sitorus"
    description = "Wind Model Python FMU Implementation"
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        
        # =========================
        # Parameters (fixed)
        # =========================
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
        
        self.manual_mean_wind_speed                     = True
        
        # What to do on failure
        # If True: outputs become 0 and valid=False
        # If False: hold last outputs and valid=False
        self.fail_outputs_zero = True
        
        # =========================
        # Inputs (time-varying commands from RL/master)
        # =========================
        self.mean_wind_speed                            = 0.0
        self.mean_wind_direction                        = 0.0
        
        # =========================
        # Outputs
        # =========================
        self.wind_speed                                 = 0.0
        self.wind_direction                             = 0.0
        self.wind_valid                                 = True
        
        # Internal state
        self.spectrum_is_computed                       = False
        self.pre_compute                                = False
        self.rng                                        = None
        self.Ubar                                       = 0.0
        self.dir                                        = 0.0
        self.mu_Ubar                                    = 0.0
        self.mu_dir                                     = 0.0
        self.sigma_Ubar                                 = 0.0
        self.sigma_dir                                  = 0.0
        
        # Gust internals
        self.f                                          = None
        self.df                                         = None
        self.a                                          = None
        self.theta                                      = None
        
        # =========================
        # Registration
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
        self.register_variable(Boolean("manual_mean_wind_speed", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Boolean("fail_outputs_zero", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))

        # Inputs (recommended for RL-driven mid-sim changes)
        self.register_variable(Real("mean_wind_speed", causality=Fmi2Causality.input))
        self.register_variable(Real("mean_wind_direction", causality=Fmi2Causality.input))

        # Outputs
        self.register_variable(Real("wind_speed", causality=Fmi2Causality.output))
        self.register_variable(Real("wind_direction", causality=Fmi2Causality.output))
        self.register_variable(Boolean("wind_valid", causality=Fmi2Causality.output))

    # =========================
    # Helpers
    # =========================
    
    def wrap_pi(self, a: float) -> float:
        return (a + np.pi) % (2.0 * np.pi) - np.pi
    
    
    # NORSOK Spectrum  
    def _norsok(self, f):
        # S(f) = 320 (U10/10)^2 (z/10)^0.45 / (1 + x^n)^(5/(3n))
        # x = 172 f (z/10)^(2/3) (U10/10)^(-3/4), n = 0.468
        n = 0.468
        x = 172.0 * f * (self.z/10.0)**(2.0/3.0) * (self.U10/10.0)**(-3.0/4.0)
        return 320.0 * (self.U10/10.0)**2 * (self.z/10.0)**0.45 / (1.0 + x**n)**(5.0/(3.0*n))
    
        
    def pre_compute_initial_parameters(self):
        # Random seed
        self.rng    = np.random.default_rng(self.seed)  # private RNG
        
        # Map user params to internal names safely
        self.z = float(self.wind_evaluation_height)
        self.kappa = float(self.kappa_parameter)

        # Frequency grid
        self.f_min  = float(self.minimum_wind_gust_frequency)
        self.f_max  = float(self.maximum_wind_gust_frequency)
        self.N_f    = int(self.wind_gust_frequency_discrete_unit_count)
        self.f      = np.linspace(self.f_min, self.f_max, self.N_f)   # Hz
        self.df     = self.f[1] - self.f[0]
        
        # Get the initial mean wind speed, available with two mode:
        # 1. Mean wind at height z
        # 2. Manual mean wind at initialization
        if bool(self.manual_mean_wind_speed):
            self.Ubar = float(self.initial_mean_wind_speed)
        else:
            # Guard kappa and z
            k           = max(1e-9, self.kappa)
            z           = max(1e-6, self.z)
            z0          = float(10.0 * np.exp(-2.0/(5.0*np.sqrt(k))))
            self.Ubar   = float(self.U10 * (2.5*np.sqrt(k)) * np.log(z / z0))
        self.mu_Ubar    = float(self.mean_wind_speed_decay_rate)
        self.sigma_Ubar = float(self.mean_wind_speed_standard_deviation)
            
        # Get the initial wind direction
        self.dir        = self.wrap_pi(float(self.initial_wind_direction))
        self.mu_dir     = float(self.wind_direction_decay_rate)
        self.sigma_dir  = float(self.wind_direction_standard_deviation)
        
        # Wind speed bound
        self.Ubar_min   = self.minimum_mean_wind_speed
        self.Ubar_max   = self.maximum_mean_wind_speed
        
        # Precompute gust amplitudes & phases (fixed), keep running angles
        if (not self.spectrum_is_computed) and (self.f is not None):
            S = self._norsok(self.f)
            self.a = np.sqrt(2.0 * S * self.df)                  # (N_f,)
            self.phi0 = 2*np.pi*self.rng.random(self.N_f)        # fixed phases
            self.theta = self.phi0.copy()                        # running angles
            self.spectrum_is_computed = True
            
    
    def compute_wind_gust(self, step_size):
        # advance phases by 2π f dt and sum components
        self.theta += 2*np.pi*self.f*step_size
        return float(np.sum(self.a * np.cos(self.theta)))

    
    # Exact OU step (stable for variable dt)
    # m -> mean value
    # X_{k+1} = m + (X_k - m) * exp(-mu dt) + sigma * sqrt((1-exp(-2 mu dt))/(2 mu)) * N(0,1)
    def ou_exact_step(self, x, m, mu, sigma, dt):
        if dt <= 0.0:
            return x  # nothing to do

        if mu <= 0.0:
            # Pure random walk around mean with drift disabled:
            # x_{k+1} = x_k + sigma*sqrt(dt)*N(0,1)  (no reversion)
            return x + sigma * np.sqrt(dt) * self.rng.normal(0.0, 1.0)

        a = np.exp(-mu * dt)
        var = (1.0 - np.exp(-2.0 * mu * dt)) / (2.0 * mu)
        return m + (x - m) * a + sigma * np.sqrt(var) * self.rng.normal(0.0, 1.0)
    

    # =========================
    # Main stepping
    # =========================       
    def do_step(self, current_time: float, step_size: float) -> bool:
        """"
        vel_mean = action outputed by RL agent. 
        [vel_mean > 0], negative value will automatically reversed to positive
        
        dir_mean = action outputed by RL agent.
        
        Mean-driven OU:
          dUbar = -mu_Ubar (Ubar - mean_speed) dt + sigma_Ubar dW
          dpsi  = -mu_dir wrap(psi - mean_dir) dt + sigma_dir dW
        """
        try:
            # Pre-compute the initial parameters
            if not self.pre_compute:
                self.pre_compute_initial_parameters()
                self.pre_compute = True
                
            dt = float(step_size)
            if dt <= 0.0:
                # Keep outputs/state unchanged; still "valid"
                self.wind_valid = True
                return True

            # Read inputs (master can keep them constant for many steps and change anytime)
            mean_speed  = 0.0 if self.mean_wind_speed is None else float(self.mean_wind_speed)
            mean_dir    = 0.0 if self.mean_wind_direction is None else float(self.mean_wind_direction)
            
            ## Compute wind speed
            # Ubar
            mean_speed  = abs(mean_speed)   # Nonnegative value
            self.Ubar   = self.ou_exact_step(self.Ubar, mean_speed, self.mu_Ubar, self.sigma_Ubar, dt)
            self.Ubar   = np.clip(self.Ubar, self.Ubar_min, self.Ubar_max)
            
            # Wind gust
            Ug          = 0.0
            if self.spectrum_is_computed and (self.f is not None):
                Ug          = self.compute_wind_gust(dt)
                
            # Total wind speed
            wind_speed  = self.Ubar + Ug
            if self.clip_speed_nonnegative:
                wind_speed = max(0.0, wind_speed)
                
            ## Compute wind direction
            # Compute direction deviation
            mean_dir    = self.wrap_pi(mean_dir)
            err         = self.wrap_pi(self.dir - mean_dir)
            
            # Integrate the OU-step for the error
            err_new     = self.ou_exact_step(err, 0.0, self.mu_dir, self.sigma_dir, dt)     # Error OU-step on error mean 0
            
            # Get the overall wind direction
            self.dir    = self.wrap_pi(mean_dir + err_new)
            
            # Assign the results to the output variables
            self.wind_speed     = float(wind_speed)
            self.wind_direction = float(self.dir)
            self.wind_valid     = True
        
        except Exception as e:
            print(f"[WindModel] Exception t={current_time}, dt={step_size}: {type(e).__name__}: {e}")
            print(traceback.format_exc())
            
            self.current_valid = False
            if self.fail_outputs_zero:
                self.current_speed = 0.0
                self.current_direction = 0.0
            # else: hold last outputs (do nothing)
            
        
        return True