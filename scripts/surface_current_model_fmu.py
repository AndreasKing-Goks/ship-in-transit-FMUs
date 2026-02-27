"""
Surface Current Model Python FMU implementation.
Mean-driven Ornstein–Uhlenbeck (OU) / Gauss–Markov process for surface current.

- RL (or external controller) drives the *mean* current speed and direction.
- The FMU outputs a stochastic current that reverts toward those means with inertia.

Authors : Andreas R.G. Sitorus
Date    : February 2026
"""

from pythonfmu import Fmi2Causality, Fmi2Slave, Fmi2Variability, Real, Integer, Boolean
import numpy as np
import traceback


class SurfaceCurrentModel(Fmi2Slave):

    author = "Andreas R.G. Sitorus"
    description = "Surface Current Model Python FMU Implementation"

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # =========================
        # Parameters (fixed)
        # =========================
        self.seed                                       = 0

        self.initial_current_speed                      = 0.0
        self.current_speed_decay_rate                   = 0.0        # mu_vel [1/s]
        self.current_speed_standard_deviation           = 0.0 # sigma_vel [units of speed / sqrt(s)] for SDE

        self.initial_current_direction_deg              = 0.0
        self.current_direction_deg_decay_rate           = 0.0       # mu_dir [1/s]
        self.current_direction_deg_standard_deviation   = 0.0 # sigma_dir [rad / sqrt(s)] for SDE

        self.clip_speed_nonnegative                     = True

        # What to do on failure
        # If True: outputs become 0 and valid=False
        # If False: hold last outputs and valid=False
        self.fail_outputs_zero                          = True

        # =========================
        # Inputs (time-varying commands from RL/master)
        # =========================
        self.mean_current_speed                         = 0.0
        self.mean_current_direction_deg                 = 0.0

        # =========================
        # Outputs
        # =========================
        self.current_speed                              = 0.0
        self.current_direction_rad                      = 0.0
        self.current_direction_deg                      = 0.0
        self.current_valid                              = True

        # Internal state
        self.pre_compute                                = False
        self.rng                                        = None
        self.vel                                        = 0.0
        self.dir                                        = 0.0
        self.mu_vel                                     = 0.0
        self.mu_dir                                     = 0.0
        self.sigma_vel                                  = 0.0
        self.sigma_dir                                  = 0.0

        # =========================
        # Registration
        # =========================

        # Fixed parameters
        self.register_variable(Integer("seed", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))

        self.register_variable(Real("initial_current_speed", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("current_speed_decay_rate", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("current_speed_standard_deviation", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))

        self.register_variable(Real("initial_current_direction_deg", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("current_direction_deg_decay_rate", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Real("current_direction_deg_standard_deviation", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))

        self.register_variable(Boolean("clip_speed_nonnegative", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))
        self.register_variable(Boolean("fail_outputs_zero", causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed))

        # Inputs (recommended for RL-driven mid-sim changes)
        self.register_variable(Real("mean_current_speed", causality=Fmi2Causality.input))
        self.register_variable(Real("mean_current_direction_deg", causality=Fmi2Causality.input))

        # Outputs
        self.register_variable(Real("current_speed", causality=Fmi2Causality.output))
        self.register_variable(Real("current_direction_rad", causality=Fmi2Causality.output))
        self.register_variable(Real("current_direction_deg", causality=Fmi2Causality.output))
        self.register_variable(Boolean("current_valid", causality=Fmi2Causality.output))

    # =========================
    # Helpers
    # =========================
    def wrap_pi(self, a: float) -> float:
        return (a + np.pi) % (2.0 * np.pi) - np.pi

    def pre_compute_initial_parameters(self):
        # RNG
        self.rng        = np.random.default_rng(int(self.seed))

        # State
        self.vel        = float(self.initial_current_speed)
        self.dir        = self.wrap_pi(float(np.deg2rad(self.initial_current_direction_deg)))

        # OU parameters
        self.mu_vel     = float(self.current_speed_decay_rate)
        self.mu_dir     = float(self.current_direction_deg_decay_rate)

        self.sigma_vel  = float(self.current_speed_standard_deviation)
        self.sigma_dir  = float(np.deg2rad(self.current_direction_deg_standard_deviation))

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

        a       = np.exp(-mu * dt)
        var     = (1.0 - np.exp(-2.0 * mu * dt)) / (2.0 * mu)
        return m + (x - m) * a + sigma * np.sqrt(var) * self.rng.normal(0.0, 1.0)

    # =========================
    # Main stepping
    # =========================
    def do_step(self, current_time: float, step_size: float) -> bool:
        """
        Mean-driven OU:
          dV   = -mu_vel (V - mean_speed) dt + sigma_vel dW
          dpsi = -mu_dir wrap(psi - mean_dir) dt + sigma_dir dW
        """
        try:
            if not self.pre_compute:
                self.pre_compute_initial_parameters()
                self.pre_compute = True

            dt = float(step_size)
            if dt <= 0.0:
                # Keep outputs/state unchanged; still "valid"
                self.current_valid = True
                return True

            # Read inputs (master can keep them constant for many steps and change anytime)
            mean_speed  = 0.0 if self.mean_current_speed is None else float(self.mean_current_speed)
            mean_dir    = 0.0 if self.mean_current_direction_deg is None else float(np.deg2rad(self.mean_current_direction_deg))

            # If you truly want speed mean always nonnegative:
            mean_speed  = abs(mean_speed)

            # --- Speed OU to mean ---
            self.vel    = self.ou_exact_step(self.vel, mean_speed, self.mu_vel, self.sigma_vel, dt)
            if self.clip_speed_nonnegative:
                self.vel = max(0.0, self.vel)

            # --- Direction OU on circle (revert using shortest angular error) ---
            # We update psi by applying OU on the *error* around the mean.
            mean_dir    = self.wrap_pi(mean_dir)
            err         = self.wrap_pi(self.dir - mean_dir)

            # OU exact step on error toward 0
            err_new     = self.ou_exact_step(err, 0.0, self.mu_dir, self.sigma_dir, dt)
            self.dir    = self.wrap_pi(mean_dir + err_new)

            # outputs
            self.current_speed          = float(self.vel)
            self.current_direction_rad  = float(self.dir)
            self.current_direction_deg  = float(np.rad2deg(self.dir))
            self.current_valid          = True

        except Exception as e:
            print(f"[SurfaceCurrentModel] Exception t={current_time}, dt={step_size}: {type(e).__name__}: {e}")
            print(traceback.format_exc())

            self.current_valid = False
            if self.fail_outputs_zero:
                self.current_speed          = 0.0
                self.current_direction_rad  = 0.0
                self.current_direction_deg  = 0.0
            # else: hold last outputs (do nothing)

        return True