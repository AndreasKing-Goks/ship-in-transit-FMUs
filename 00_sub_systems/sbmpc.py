import numpy as np, math
from dataclasses import dataclass, field
from simulator.ship_in_transit.utils.sbmpc_misc import wrap_angle_to_pmpi, Obstacle, ShipLinearModel

@dataclass
class SBMPCParams:
    """Parameters for the SB-MPC algorithm."""

    P_: float = 1.0  # weights the importance of time until the event of collision occurs
    Q_: float = 4.0  # exponent to satisfy colregs rule 16
    D_INIT_: float = 2000.0  # should be >= D_CLOSE   # distance to an obstacle to activate sbmpc [m]
    D_CLOSE_: float = 2000.0  # distance for an nearby obstacle [m]
    D_SAFE_: float = 1000.0  # distance of safety zone [m]
    K_COLL_: float = 1e-6  # Weight for cost of collision --> C_i^k = K_COLL * |v_os - v_i^k|^2
    PHI_AH_: float = np.deg2rad(68.5)  # colregs angle - ahead [deg]
    PHI_OT_: float = np.deg2rad(68.5)  # colregs angle - overtaken [deg]
    PHI_HO_: float = np.deg2rad(22.5)  # colregs angle -  head on [deg]
    PHI_CR_: float = np.deg2rad(68.5)  # colregs angle -  crossing [deg]
    KAPPA_: float = 0.0 # 10.0  # Weight for cost of COLREGs compliance (Rules 14 & 15, if both are satisfied it implies 13 is also satisfied)
    K_P_: float = 25  # Weight for penalizing speed offset
    K_CHI_: float = 30  # Weight for penalizing heading offset
    K_DP_: float = 20  # Weight for penalizing changes in speed offset
    K_DCHI_SB_: float = 20  # Weight for penalizing changes in heading offset in StarBoard situation
    K_DCHI_P_: float = 30  # Weight for penalizing changes in heading offset in Port situation

    # K_DCHI is greater in port than in starboard in compliance with COLREGS rules 14, 15 and 17.

    P_ca_last_: float = 1.0  # last control change
    Chi_ca_last_: float = 0.0  # last course change

    Chi_ca_: np.array = field(
        default_factory=lambda: np.deg2rad(
            np.array([-30.0, -20.0, -10.0, 0.0, 10.0, 20.0, 30.0])
        )
    )  # control behaviors - course offset [deg]
    P_ca_: np.array = field(default_factory=lambda: np.array([0.4, 0.6, 0.8, 1.0]))  # control behaviors - speed factor

    def to_dict(self):
        output = {
            "P_": self.P_,
            "Q_": self.Q_,
            "D_INIT_": self.D_INIT_,
            "D_CLOSE_": self.D_CLOSE_,
            "D_SAFE_": self.D_SAFE_,
            "K_COLL_": self.K_COLL_,
            "PHI_AH_": self.PHI_AH_,
            "PHI_OT_": self.PHI_OT_,
            "PHI_HO_": self.PHI_HO_,
            "PHI_CR_": self.PHI_CR_,
            "KAPPA_": self.KAPPA_,
            "K_P_": self.K_P_,
            "K_CHI_": self.K_CHI_,
            "K_DP_": self.K_DP_,
            "K_DCHI_SB_": self.K_DCHI_SB_,
            "K_DCHI_P_": self.K_DCHI_P_,
            "P_ca_last": self.P_ca_last_,
            "Chi_ca_last": self.Chi_ca_last_,
            "Chi_ca_": self.Chi_ca_,
            "P_ca_": self.P_ca_,
        }
        return output

    @classmethod
    def from_dict(cls, data: dict):
        output = SBMPCParams(
            P_=data["P_"],
            Q_=data["Q_"],
            D_INIT_=data["D_INIT_"],
            D_CLOSE_=data["D_CLOSE_"],
            D_SAFE_=data["D_SAFE_"],
            K_COLL_=data["K_COLL_"],
            PHI_AH_=data["PHI_AH_"],
            PHI_OT_=data["PHI_OT_"],
            PHI_HO_=data["PHI_HO_"],
            PHI_CR_=data["PHI_CR_"],
            KAPPA_=data["KAPPA_"],
            K_P_=data["K_P_"],
            K_CHI_=data["K_CHI_"],
            K_DP_=data["K_DP_"],
            K_DCHI_SB_=data["K_DCHI_SB_"],
            K_DCHI_P_=data["K_DCHI_P_"],
            P_ca_last_=data["P_ca_last_"],
            Chi_ca_last_=data["Chi_ca_last_"],
            Chi_ca_=data["Chi_ca_"],
            P_ca_=data["P_ca_"],
        )
        return output
    

class SBMPC:
    def __init__(self, tf: float, dt: float, config: SBMPCParams = None) -> None:
        # TODO: Make SBMPC able to handle different sampling time than the actual system -> probably easy since all shipmodels are linear ?

        # NB os_ship: copy of own ship initialized class
        self.T_ = tf  # prediction horizon [s]
        self.DT_ = dt  # time step [s]
        self.n_samp = int(self.T_ / self.DT_)  # number of samplings

        self.cost_ = np.inf

        self.ownship = ShipLinearModel(self.T_, self.DT_)
        # print("OWN SHIP: ", self.ownship)

        if config:
            self._params = config
        else:
            self._params = SBMPCParams()

        self.active = False # Flag indicating whether or not we are evaluating best control behaviour

        # print(f"SBMPC created successfully with T = {self.T_}, dt = {self.DT_} :)")

    def get_optimal_ctrl_offset(
        self,
        u_d: float,
        chi_d: float,
        os_state: np.ndarray,
        do_list: list[tuple[int, np.ndarray, np.ndarray, float, float]]
        # enc: senc.ENC,
    ) -> tuple[float, float]:
        """Calculates the optimal control offset for the own ship using the SB-MPC algorithm.

        Args:
            u_d (float): Nominal surge speed reference for the own ship.
            chi_d (float): Nominal course reference for the own ship.
            os_state (np.ndarray): Current state of the own ship.
            do_list (List[Tuple[int, np.ndarray, np.ndarray, float, float]]): List of tuples containing the dynamic obstacle info
            enc (senc.ENC): Electronic navigational chart.

        Returns:
            Tuple[float, float]: Optimal control offset to the own ship nominal LOS references, (speed factor, course offset).
        """
        cost = np.inf
        cost_i = 0
        self.active = False
        d = np.zeros(2)

        if do_list is None:
            u_os_best = 1
            chi_os_best = 0
            self._params.P_ca_last_ = 1
            self._params.Chi_ca_last_ = 0
            return u_os_best, chi_os_best
        else:
            obstacles = []
            n_obst = len(do_list)
            for obs_state in do_list:
                obstacle = Obstacle(obs_state, self.T_, self.DT_)
                obstacles.append(obstacle)

        # check if obstacles are within init range
        for obs in obstacles:
            d[0] = obs.x_[0] - os_state[0]
            d[1] = obs.y_[0] - os_state[1]
            
            if np.linalg.norm(d) < self._params.D_INIT_:
                self.active = True

        if not self.active:
            u_os_best = 1
            chi_os_best = 0
            self._params.P_ca_last_ = 1
            self._params.Chi_ca_last_ = 0
            return u_os_best, chi_os_best

        for i in range(len(self._params.Chi_ca_)):
            for j in range(len(self._params.P_ca_)):
                self.ownship.linear_pred(os_state, u_d * self._params.P_ca_[j], chi_d + self._params.Chi_ca_[i])

                cost_i = -1
                for k in range(n_obst):
                    cost_k = self.cost_func(self._params.P_ca_[j], self._params.Chi_ca_[i], obstacles[k])
                    if cost_k > cost_i: # Parmis tous les obstacles, on garde que celui qui amène le pire résultat, pour chaque combinaison de P, Chi
                        cost_i = cost_k
                if cost_i < cost: # Ensuite parmis tous ces worst case, on garde la combinaison de P, Chi qui donne le moins pire résultat
                    cost = cost_i
                    u_os_best = self._params.P_ca_[j]
                    chi_os_best = self._params.Chi_ca_[i]
        
        self._params.P_ca_last_ = u_os_best
        self._params.Chi_ca_last_ = chi_os_best

        return u_os_best, chi_os_best
    
    def is_stephen_useful(self) -> bool:
        return self.active

    def cost_func(self, P_ca: float, Chi_ca: float, obstacle:Obstacle):
        obs_l = obstacle.l
        obs_w = obstacle.w
        os_l = self.ownship.l
        os_w = self.ownship.w

        d, v_o, v_s = np.zeros(2), np.zeros(2), np.zeros(2)
        self.combined_radius = os_l + obs_l
        d_safe = self._params.D_SAFE_
        d_close = self._params.D_CLOSE_
        H0, H1, H2 = 0, 0, 0
        cost = 0
        t = 0
        t0 = 0

        for i in range(self.n_samp):

            t += self.DT_

            d[0] = obstacle.x_[i] - self.ownship.x_[i]
            d[1] = obstacle.y_[i] - self.ownship.y_[i]
            dist = np.linalg.norm(d)

            R = 0
            C = 0
            mu = 0

            if dist < d_close:
                v_o[0] = obstacle.u_[i]
                v_o[1] = obstacle.v_[i]
                v_o = self.rot2d(obstacle.psi_, v_o) # --> speed of obstacle in world frame

                v_s[0] = self.ownship.u_[i]
                v_s[1] = self.ownship.v_[i]
                v_s = self.rot2d(self.ownship.psi_[i], v_s) # --> speed of ownship in world frame

                psi_o = wrap_angle_to_pmpi(obstacle.psi_)
                phi = wrap_angle_to_pmpi(math.atan2(d[1], d[0]) - self.ownship.psi_[i] - math.pi/2) # Difference between os heading and direction towards obstacle
                psi_rel = wrap_angle_to_pmpi(psi_o - self.ownship.psi_[i]) # relative heading of obstacle w.r.t ownship's heading

                if phi < self._params.PHI_AH_: # If the ship's heading is oriented too much towards the obstacle (i.e. in the AHEAD SECTOR)
                    d_safe_i = d_safe + os_l / 2 # Then we increase the safe distance by ownship's length
                elif phi > self._params.PHI_OT_: # If the ship's heading is not oriented towards the obstacle (i.e. in the OVERTAKING SECTOR)
                    d_safe_i = 0.5 * d_safe + os_l / 2 #  reduce safe distance
                else:
                    d_safe_i = d_safe + os_w / 2

                phi_o = wrap_angle_to_pmpi(math.atan2(-d[1], -d[0]) - obstacle.psi_ + math.pi/2) 

                if phi_o < self._params.PHI_AH_:
                    d_safe_i = d_safe + obs_l / 2
                elif phi_o > self._params.PHI_OT_:
                    d_safe_i = 0.5 * d_safe + obs_l / 2
                else:
                    d_safe_i = d_safe + obs_w / 2


                
                if (np.dot(v_s, v_o)) > np.cos(np.deg2rad(self._params.PHI_OT_)) * np.linalg.norm(v_s) * np.linalg.norm(
                    v_o
                ) and np.linalg.norm(v_s) > np.linalg.norm(v_o):
                    d_safe_i = d_safe + os_l / 2 + obs_l / 2 # --> Increases safety distance

                if dist < d_safe_i:
                    R = (1 / (abs(t - t0) ** self._params.P_)) * (d_safe / dist) ** self._params.Q_
                    k_coll = self._params.K_COLL_ * os_l * obs_l
                    C = k_coll * np.linalg.norm(v_s - v_o) ** 2

                # Overtaken by obstacle
                # Those conditions checks if the ship is being OVERTAKEN by the obstacle
                # first condition checks if the angle between their speed is greater than the overtaking angle (PHI_OT)
                # second condition checks if the ship has greater speed than the obstacle
                OT = (np.dot(v_s, v_o)) > np.cos(np.deg2rad(self._params.PHI_OT_)) * np.linalg.norm(
                    v_s
                ) * np.linalg.norm(v_o) and np.linalg.norm(v_s) < np.linalg.norm(v_o)

                # Obstacle on starboard side
                SB = phi >= 0

                # Obstacle Head-on
                HO = (
                    np.linalg.norm(v_o) > 0.05
                    and (np.dot(v_s, v_o))
                    < -np.cos(np.deg2rad(self._params.PHI_HO_)) * np.linalg.norm(v_s) * np.linalg.norm(v_o)
                    and (np.dot(v_s, v_o)) > np.cos(np.deg2rad(self._params.PHI_AH_)) * np.linalg.norm(v_s)
                )

                # Crossing situation
                CR = (np.dot(v_s, v_o)) < np.cos(np.deg2rad(self._params.PHI_CR_)) * np.linalg.norm(
                    v_s
                ) * np.linalg.norm(v_o) and (SB and psi_rel < 0)

                mu = (SB and HO) or (CR and not OT)

                # if i==0 and P_ca==1.0 and Chi_ca==0.0:
                #     print(f"os_psi:{self.ownship.psi_[0]} \t obs_psi:{obstacle.psi_} \t OT:{OT} \t SB:{SB} \t HO:{HO} \t CR:{CR}")

            H0 = C * R + self._params.KAPPA_ * mu
            if H0 > H1: # H1 is basically the maximum cost of collision along the trajectory
                H1 = H0

        H2 = (
            self._params.K_P_ * (1 - P_ca)
            + self._params.K_CHI_ * Chi_ca**2
            + self.delta_P(P_ca)
            + self.delta_Chi(Chi_ca)
        )
        cost = H1 + H2
        return cost

    def delta_P(self, P_ca):
        return self._params.K_DP_ * abs(self._params.P_ca_last_ - P_ca)

    def delta_Chi(self, Chi_ca):
        d_chi = Chi_ca - self._params.Chi_ca_last_
        if d_chi > 0:
            return self._params.K_DCHI_SB_ * d_chi**2
        elif d_chi < 0:
            return self._params.K_DCHI_P_ * d_chi**2
        else:
            return 0

    def rot2d(self, yaw: float, vec: np.ndarray):
        R = np.array([[-np.sin(yaw), np.cos(yaw)], [np.cos(yaw), np.sin(yaw)]])
        return R @ vec


