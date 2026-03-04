import numpy as np
import copy
from scipy.special import factorial
from typing import NamedTuple

class WaveModelConfiguration(NamedTuple):
    minimum_wave_frequency: float
    maximum_wave_frequency: float
    wave_frequency_discrete_unit_count: int
    minimum_spreading_angle: float
    maximum_spreading_angle: float
    spreading_angle_discrete_unit_count: int
    spreading_coefficient: int
    rho:float
    timestep_size: float

class JONSWAPWaveModel:
    '''
    This class defines a wave model based on JONSWAP wave spectrum
    with wave spreading function.
    '''
    def __init__(self, config: WaveModelConfiguration, seed=None):
        '''
        Parameters:
        -----------
        omega_vec : float or np.array
            Angular frequencies [rad/s].
        psi_vec: float or np.array
            Discretized spreading angle [rad]
        k_vec : float or np.array
            Vector of wave numbers.
        gamma : float
            Peak enhancement factor (default ~3.3 for JONSWAP).
        g : float
            Gravity [m/s^2].
        s : int
            spreading parameter. 
                - s = 1 recommended by ITTC
                - s = 2 recommended by ISSC
        w_min: float
            The smallest available wave frequency.
        w_max: float
            The biggest available wave frequency.
        N_omega: int
            Discrete units count for the wave frequency discretization.
        psi_min: float
            The low bound for wave spread.
        psi_max: float
            The high bound for wave spread.
        N_psi: int
            Discrete units count for the wave spreading discretization.
        dt: float
            Time step size for time integration.
        '''
        # Random seed
        self.rng = np.random.default_rng(seed)  # private RNG
        
        # Config
        self.config = config
        
        # Self parameters
        self.g = 9.81
        self.s = config.spreading_coefficient
        self.w_min = config.minimum_wave_frequency
        self.w_max = config.maximum_wave_frequency
        self.N_omega = config.wave_frequency_discrete_unit_count
        self.psi_min = config.minimum_spreading_angle
        self.psi_max = config.maximum_spreading_angle
        self.N_psi = config.spreading_angle_discrete_unit_count
        self.dt = config.timestep_size
        self.rho = config.rho
        
        # Vector for each wave across all frequencies
        self.omega_vec = np.linspace(self.w_min, self.w_max, self.N_omega)
        self.domega = self.omega_vec[1] - self.omega_vec[0]
        
        # Vector for wave numbers
        self.k_vec = self.omega_vec**2 / self.g
        
        # Vector for each wave across discretized spreading direction
        self.psi_vec = np.linspace(self.psi_min, self.psi_max, self.N_psi)
        self.dpsi = self.psi_vec[1] - self.psi_vec[0]
        
        # Vector for randp, phases for each wave across all frequencies
        self.theta = 2.0 * np.pi * self.rng.random((self.N_omega, self.N_psi))    # (Nw, Nd)
            
        # Record of the initial parameters
        self.record_initial_parameters()
        
    def set_seed(self, seed: int | None):
        # Allow reseeding at any time
        self.rng = np.random.default_rng(seed)
        
    def jonswap_spectrum(self, Hs, Tp, omega_vec):
        '''
        Returns S_eta(omega_vec): wave spectrum [m^2 s] across  all predetermined 
        wave frequencies using a JONSWAP formula in Faltinsen (1993).
        
        '''
        # Peak frequency
        wp = 2.0 * np.pi / Tp
        
        # Clip the wp based on Hs to keep within the validity area of the spectrum
        wp = np.clip(wp, 1.25/np.sqrt(Hs), 1.75/np.sqrt(Hs))
        
        # Alpha
        alpha = 0.2 * Hs**2 * wp**4 / self.g**2
        
        # Sigma (check across all frequency, then assign the sigma to each frequency)
        sigma = np.where(omega_vec <= wp, 0.07, 0.09)
        
        # Getting the gamma based on DNV GL [Sørensen (2018)]
        k = (2 * np.pi) / (wp * np.sqrt(Hs))
        
        if k <= 3.6:
            gamma = 5
        elif k <= 5.0:
            gamma = np.exp(5.75 - 1.15 * k)
        elif k > 5.0:
            gamma = 1
            
        # JONSWAP core
        gamma_exp = np.exp(- (omega_vec - wp)**2 / (2 * sigma**2 * wp**2))
        Sj = alpha * self.g**2 / omega_vec**5 * np.exp(-1.25*(wp/omega_vec)**4) * gamma**gamma_exp
        
        # Ensure non negative (negative energy density has no meaning at all)
        return np.maximum(Sj, 0.0)
    
    def spreading_function(self, psi_0, s, psi_vec):
        '''
        Returns D(psi): Spreading factor to spread the wave energy around the 
        mean wave direction
        '''
        delta_psi = psi_vec - psi_0
        
        spread_factor = (2**(2*s - 1) * factorial(s) * factorial(s-1)) / (np.pi * factorial(2*s - 1))

        # Core definition
        D = np.where(
            np.abs(delta_psi) < np.pi/2,
            spread_factor * np.cos(delta_psi)**(2*s),
            0.0
        )
        return D
    
    def get_direct_wave_force(self, ship_speed, psi_ship,
                          ship_length, ship_breadth, ship_draft,
                          Hs, Tp, psi_0, 
                          omega_vec=None, psi_vec=None, s=None):
        '''
        Parameters:
        ship_speed: float
            Ship forward speed
        psi_ship: float
            Ship heading
        A_proj: flowat
            Projected ship section area at the waterline
        Hs : float
            Significant wave height [m].
        Tp : float
            Peak period [s].
        psi_0 : float
            Mean wave direction [rad]
            
        Returns (F_wx, F_wy): First order wave load computed with Froude-Krylof force approximation. [N]
        '''
        
        if omega_vec is None: omega_vec = self.omega_vec      # (Nw,)
        if psi_vec is None: psi_vec = self.psi_vec          # (Nd,)
        if s is None: s = self.s
        
        # Compute wave spectrum and the spreading function
        S_w = self.jonswap_spectrum(Hs, Tp, omega_vec)      # (Nw,)
        D_psi = self.spreading_function(psi_0, s, psi_vec)  # (Nd,)
        # Normalize the D to integrate to 1 over psi
        # So that the sum over D(psi) dpsi = 1
        D_psi = D_psi / (D_psi.sum() * self.dpsi)           # (Nd,)
        
        # Component elevation amplitudes
        a_eta = np.sqrt(2.0 * np.outer(S_w, D_psi) * self.domega * self.dpsi)   # (Nw, Nd)
        
        # Encounter correction [Forward speed effect in Faltinsen (1993)]
        beta = psi_vec[None, :] - psi_ship                                       # (1, Nd)
        omega_e = omega_vec[:, None] - self.k_vec[:, None] * ship_speed * np.cos(beta)    # (Nw, 1) - (Nw, 1)*(1, Nd) = (Nw, Nd)
        
        # Approximation of oblique wave
        beta_0 = psi_0 - psi_ship
        A_proj = (ship_breadth * np.cos(beta_0) + ship_length * np.sin(beta_0)) * ship_draft
        
        # Froude-Krylov flat-plate force amplitude
        F0 = self.rho * self.g * a_eta * A_proj
        
        # Direction unit vectors
        cx = np.cos(beta)  # (1, Nd)
        cy = np.sin(beta)  # (1, Nd)
        
        # Fx(t) = sum_{i,j} F0[i,j] * cos(omega_e[i,j]*t + phi[i,j]) * cos(theta_j)
        arg    = omega_e * self.dt + self.theta
        cosarg = np.cos(arg)                                                      # (Nw, Nd)
        
        # advance phases by Δt
        self.theta = (self.theta + omega_e * self.dt) % (2*np.pi)
        # eta = A cos(omega_e*dt + psi0) = A cos(theta)
        # Discrete step t_k = k * dt
        # Instead of tracking the k, we can advances the theta by:
        # Adding the theta with another omega_e*dt, divide by a full sinusoidal cycle of 2*pi,
        # then get the remain. This remain is the advances of theta within the [0, 2*pi)

        # Component forces along x,y per (i,j)
        Fx_ij = F0 * cosarg * cx   # (Nw, Nd)
        Fy_ij = F0 * cosarg * cy   # (Nw, Nd)

        # ---- Lever arms for yaw moment (about CG) ----
        # Smoothly blend bow/stern (±L/2) and port/stbd (±B/2) with heading weight
        def r_cp_from_beta(beta, L, B):
            wx = np.abs(np.cos(beta))
            wy = np.abs(np.sin(beta))
            wsum = wx + wy + 1e-12  # avoid zero
            rx = 0.5 * L * np.sign(np.cos(beta)) * (wx / wsum)
            ry = 0.5 * B * np.sign(np.sin(beta)) * (wy / wsum)
            return rx, ry

        rx, ry = r_cp_from_beta(beta, ship_length, ship_breadth)  # (1, Nd)

        # Yaw moment sum Mz = r_x*Fy - r_y*Fx (sum over freq & dir)
        Mz_t = np.sum(rx * Fy_ij - ry * Fx_ij, axis=(0, 1))                 # scalar

        # Total forces (sum over freq & dir)
        Fx_t = np.sum(Fx_ij, axis=(0, 1))
        Fy_t = np.sum(Fy_ij, axis=(0, 1))

        return np.array([Fx_t, Fy_t, Mz_t])
    
    def get_wave_force_params(self, Hs, Tp, psi_0,
                              omega_vec=None, psi_vec=None, s=None):
        if omega_vec is None: omega_vec = self.omega_vec      # (Nw,)
        if psi_vec is None: psi_vec = self.psi_vec          # (Nd,)
        if s is None: s = self.s
        
        # Compute wave spectrum and the spreading function
        S_w = self.jonswap_spectrum(Hs, Tp, omega_vec)      # (Nw,)
        D_psi = self.spreading_function(psi_0, s, psi_vec)  # (Nd,)
        
        # Check if spectrum is consistent with Hs
        target_m0 = Hs**2 / 16.0
        m0_num = np.sum(S_w) * self.domega
        if m0_num > 0:
            S_w *= (target_m0 / m0_num)  # scale to match Hs
        
        # Normalize the D to integrate to 1 over psi
        # So that the sum over D(psi) dpsi = 1
        D_psi = D_psi / (D_psi.sum() * self.dpsi)           # (Nd,)
        
        return S_w, D_psi, psi_0
    
    def record_initial_parameters(self):
        '''
        Internal method to take a record of internal attributes after __init__().
        This record will be used to reset the model later without re-instantiation.
        '''
        self._initial_parameters = {
            key: copy.deepcopy(self.__dict__[key])
            for key in ['g', 's', 'w_min', 'w_max', 'N_omega',
                        'psi_min', 'psi_max', 'N_psi', 'dt', 
                        'omega_vec', 'domega', 'k_vec', 'psi_vec', 'dpsi']
            }

    def _reset_params_to_initial(self):
        for key, value in self._initial_parameters.items():
            setattr(self, key, copy.deepcopy(value))
            
        # Resample Vector for randp, phases for each wave across all frequencies
        self.theta = 2.0 * np.pi * self.rng.random((self.N_omega, self.N_psi))    # (Nw, Nd)
            
    # Gym-style reset that can accept a seed
    def reset(self, seed: int | None = None):
        if seed is not None:
            self.set_seed(seed)
        self._reset_params_to_initial()