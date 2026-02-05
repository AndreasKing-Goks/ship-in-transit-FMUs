""" 
This module provides classes to construct the ship machinery sistem to simulate.
Ship machinery includes the type of engine and diesel generators used.
"""
import copy

import numpy as np
from typing import List, NamedTuple, Union

from simulator.ship_in_transit.utils.utils import EulerInt

###################################################################################################################
###################################### CONFIGURATION FOR MACHINERY MODEL ##########################################
###################################################################################################################


class MachineryModeParams(NamedTuple):
    main_engine_capacity: float
    electrical_capacity: float
    shaft_generator_state: str
    name_tag: str


class MachineryMode:
    def __init__(self, params: MachineryModeParams):
        self.main_engine_capacity = params.main_engine_capacity
        self.electrical_capacity = params.electrical_capacity
        self.shaft_generator_state = params.shaft_generator_state
        self.name_tag = params.name_tag
        self.available_propulsion_power = 0
        self.available_propulsion_power_main_engine = 0
        self.available_propulsion_power_electrical = 0

    def update_available_propulsion_power(self, hotel_load):
        if self.shaft_generator_state == 'MOTOR':
            self.available_propulsion_power = self.main_engine_capacity + self.electrical_capacity - hotel_load
            self.available_propulsion_power_main_engine = self.main_engine_capacity
            self.available_propulsion_power_electrical = self.electrical_capacity - hotel_load
        elif self.shaft_generator_state == 'GEN':
            self.available_propulsion_power = self.main_engine_capacity - hotel_load
            self.available_propulsion_power_main_engine = self.main_engine_capacity - hotel_load
            self.available_propulsion_power_electrical = 0
        else:  # shaft_generator_state == 'off'
            self.available_propulsion_power = self.main_engine_capacity
            self.available_propulsion_power_main_engine = self.main_engine_capacity
            self.available_propulsion_power_electrical = 0

    def distribute_load(self, load_perc, hotel_load):
        total_load_propulsion = load_perc * self.available_propulsion_power
        if self.shaft_generator_state == 'MOTOR':
            load_main_engine = min(total_load_propulsion, self.main_engine_capacity)
            load_electrical = total_load_propulsion + hotel_load - load_main_engine
            load_percentage_electrical = load_electrical / self.electrical_capacity
            if self.main_engine_capacity == 0:
                load_percentage_main_engine = 0
            else:
                load_percentage_main_engine = load_main_engine / self.main_engine_capacity
        elif self.shaft_generator_state == 'GEN':
            # Here the rule is that electrical handles hotel as far as possible
            load_electrical = min(hotel_load, self.electrical_capacity)
            load_main_engine = total_load_propulsion + hotel_load - load_electrical
            load_percentage_main_engine = load_main_engine / self.main_engine_capacity
            if self.electrical_capacity == 0:
                load_percentage_electrical = 0
            else:
                load_percentage_electrical = load_electrical / self.electrical_capacity
        else:  # shaft_generator_state == 'off'
            load_main_engine = total_load_propulsion
            load_electrical = hotel_load
            load_percentage_main_engine = load_main_engine / self.main_engine_capacity
            load_percentage_electrical = load_electrical / self.electrical_capacity

        return LoadOnPowerSources(
            load_on_main_engine=load_main_engine,
            load_on_electrical=load_electrical,
            load_percentage_on_main_engine=load_percentage_main_engine,
            load_percentage_on_electrical=load_percentage_electrical
        )

class MachineryModes:
    def __init__(self, list_of_modes: List[MachineryMode]):
        self.list_of_modes = list_of_modes


class MachineryModes:
    def __init__(self, list_of_modes: List[MachineryMode]):
        self.list_of_modes = list_of_modes


class SpecificFuelConsumptionWartila6L26:
    def __init__(self):
        self.a = 128.9
        self.b = -168.9
        self.c = 246.8

    def fuel_consumption_coefficients(self):
        return FuelConsumptionCoefficients(
            a=self.a,
            b=self.b,
            c=self.c
        )

class SpecificFuelConsumptionBaudouin6M26Dot3:
    def __init__(self):
        self.a = 108.7
        self.b = -289.9
        self.c = 324.9

    def fuel_consumption_coefficients(self):
        return FuelConsumptionCoefficients(
            a=self.a,
            b=self.b,
            c=self.c
        )


class FuelConsumptionCoefficients(NamedTuple):
    a: float
    b: float
    c: float


class MachinerySystemConfiguration(NamedTuple):
    hotel_load: float
    machinery_modes: MachineryModes
    machinery_operating_mode: int
    rated_speed_main_engine_rpm: float
    linear_friction_main_engine: float
    linear_friction_hybrid_shaft_generator: float
    gear_ratio_between_main_engine_and_propeller: float
    gear_ratio_between_hybrid_shaft_generator_and_propeller: float
    propeller_inertia: float
    propeller_speed_to_torque_coefficient: float
    propeller_diameter: float
    propeller_speed_to_thrust_force_coefficient: float
    rudder_angle_to_sway_force_coefficient: float
    rudder_angle_to_yaw_force_coefficient: float
    max_rudder_angle_degrees: float
    max_rudder_rate_degree_per_s: float
    specific_fuel_consumption_coefficients_me: FuelConsumptionCoefficients
    specific_fuel_consumption_coefficients_dg: FuelConsumptionCoefficients


class WithoutMachineryModelConfiguration(NamedTuple):
    thrust_force_dynamic_time_constant: float
    rudder_angle_to_sway_force_coefficient: float
    rudder_angle_to_yaw_force_coefficient: float
    max_rudder_angle_degrees: float


class SimplifiedPropulsionMachinerySystemConfiguration(NamedTuple):
    hotel_load: float
    machinery_modes: MachineryModes
    machinery_operating_mode: int
    specific_fuel_consumption_coefficients_me: FuelConsumptionCoefficients
    specific_fuel_consumption_coefficients_dg: FuelConsumptionCoefficients
    thrust_force_dynamic_time_constant: float
    rudder_angle_to_sway_force_coefficient: float
    rudder_angle_to_yaw_force_coefficient: float
    max_rudder_angle_degrees: float
    
    
class RudderConfiguration(NamedTuple):
    rudder_angle_to_sway_force_coefficient: float
    rudder_angle_to_yaw_force_coefficient: float
    max_rudder_angle_degrees: float
    

class LoadOnPowerSources(NamedTuple):
    load_on_main_engine: float
    load_on_electrical: float
    load_percentage_on_main_engine: float
    load_percentage_on_electrical: float


###################################################################################################################
###################################################################################################################


class BaseMachineryModel:
    def __init__(self,
                 fuel_coeffs_for_main_engine: Union[FuelConsumptionCoefficients, None],
                 fuel_coeffs_for_diesel_gen: Union[FuelConsumptionCoefficients, None],
                 rudder_config: RudderConfiguration,
                 machinery_modes: Union[MachineryModes, None],
                 hotel_load: Union[float, None],
                 operating_mode: Union[int, None],
                 time_step: float):


        if machinery_modes:
            self.machinery_modes = machinery_modes
        if hotel_load:
            self.hotel_load = hotel_load  # 200000  # 0.2 MW
        if machinery_modes and hotel_load:
            self.update_available_propulsion_power()
        if operating_mode is not None and machinery_modes is not None:
            self.mode = self.machinery_modes.list_of_modes[operating_mode]

        # Set up integration
        self.int = EulerInt()  # Instantiate the Euler integrator
        self.int.set_dt(time_step)

        self.c_rudder_v = rudder_config.rudder_angle_to_sway_force_coefficient
        self.c_rudder_r = rudder_config.rudder_angle_to_yaw_force_coefficient
        self.rudder_ang_max = rudder_config.max_rudder_angle_degrees * np.pi / 180
        if fuel_coeffs_for_main_engine:
            self.fuel_coeffs_for_main_engine = fuel_coeffs_for_main_engine
            self.fuel_coeffs_for_diesel_gen = fuel_coeffs_for_diesel_gen
            self.fuel_cons_me = 0.0  # Initial fuel cons for ME
            self.fuel_cons_electrical = 0.0  # Initial fuel cons for HSG
            self.fuel_cons = 0.0  # Initial total fuel cons
            self.power_me = []  # Array for storing ME power cons. data
            self.power_hsg = []  # Array for storing HSG power cons. data
            self.me_rated = []  # Array for storing ME rated power data
            self.hsg_rated = []  # Array for storing HSG rated power data
            self.load_hist = []  # Array for storing load percentage history
            self.fuel_rate_me = []  # Array for storing ME fuel cons. rate
            self.fuel_rate_hsg = []  # Array for storing HSG fuel cons. rate
            self.fuel_me = []  # Array for storing ME fuel cons.
            self.fuel_hsg = []  # Array for storing HSG fuel cons.
            self.fuel = []  # Array for storing total fuel cons
            self.fuel_rate = []
            self.load_perc_me = []
            self.load_perc_hsg = []
            self.power_total = []
            self.power_prop = []
        
        # # Record of the initial parameters
        # self.record_initial_parameters()

    def update_available_propulsion_power(self):
        if not self.machinery_modes:
            print("Machinery modes has not been set and available propulsion power cannot be set ")
        else:
            for mode in self.machinery_modes.list_of_modes:
                mode.update_available_propulsion_power(self.hotel_load)

    def mode_selector(self, mode: int):
        if not self.machinery_modes:
            print("Mode section is not available for this machinery system")
        else:
            self.mode = self.machinery_modes.list_of_modes[mode]

    def load_perc(self, load_perc):
        """ Calculates the load percentage on the main engine and the diesel_gens based on the
            operating mode of the machinery system (MSO-mode).

            Args:
                load_perc (float): Current load on the machinery system as a fraction of the
                    total power that can be delivered by the machinery system in the current mode.
            Returns:
                load_perc_me (float): Current load on the ME as a fraction of ME MCR
                load_perc_hsg (float): Current load on the HSG as a fraction of HSG MCR
        """
        if not self.mode:
            print("Available power is not available for this machinery system")
            return 0
        load_data = self.mode.distribute_load(load_perc=load_perc, hotel_load=self.hotel_load)
        return load_data.load_percentage_on_main_engine, load_data.load_percentage_on_electrical

    @staticmethod
    def spec_fuel_cons(load_perc, coeffs: FuelConsumptionCoefficients):
        """ Calculate fuel consumption rate for engine.
        """
        rate = coeffs.a * load_perc ** 2 + coeffs.b * load_perc + coeffs.c
        return rate / 3.6e9

    def fuel_consumption(self, load_perc):
        '''
            Args:
                load_perc (float): The fraction of produced power over the online power production capacity.
            Returns:
                rate_me (float): Fuel consumption rate for the main engine
                rate_hsg (float): Fuel consumption rate for the HSG
                fuel_cons_me (float): Accumulated fuel consumption for the ME
                fuel_cons_hsg (float): Accumulated fuel consumption for the HSG
                fuel_cons (float): Total accumulated fuel consumption for the ship
        '''
        load_data = self.mode.distribute_load(load_perc=load_perc, hotel_load=self.hotel_load)
        if load_data.load_on_main_engine == 0:
            rate_me = 0
        else:
            rate_me = load_data.load_on_main_engine * self.spec_fuel_cons(
                load_data.load_percentage_on_main_engine, coeffs=self.fuel_coeffs_for_main_engine
            )

        if load_data.load_percentage_on_electrical == 0:
            rate_electrical = 0
        else:
            rate_electrical = load_data.load_on_electrical * self.spec_fuel_cons(
                load_data.load_percentage_on_electrical, coeffs=self.fuel_coeffs_for_diesel_gen
            )

        self.fuel_cons_me = self.fuel_cons_me + rate_me * self.int.dt
        self.fuel_cons_electrical = self.fuel_cons_electrical + rate_electrical * self.int.dt
        self.fuel_cons = self.fuel_cons + (rate_me + rate_electrical) * self.int.dt
        return rate_me, rate_electrical, self.fuel_cons_me, self.fuel_cons_electrical, self.fuel_cons
    
    def record_initial_parameters(self):
        ''' 
        Record internal states for later reset. 
        '''
        self._initial_state = {
            'hotel_load': copy.deepcopy(self.hotel_load) if hasattr(self, 'hotel_load') else None,
            'mode': copy.deepcopy(self.mode) if hasattr(self, 'mode') else None,
            'fuel_cons_me': self.fuel_cons_me if hasattr(self, 'fuel_cons_me') else 0.0,
            'fuel_cons_electrical': self.fuel_cons_electrical if hasattr(self, 'fuel_cons_electrical') else 0.0,
            'fuel_cons': self.fuel_cons if hasattr(self, 'fuel_cons') else 0.0,
            'power_me': copy.deepcopy(self.power_me) if hasattr(self, 'power_me') else [],
            'power_hsg': copy.deepcopy(self.power_hsg) if hasattr(self, 'power_hsg') else [],
            'me_rated': copy.deepcopy(self.me_rated) if hasattr(self, 'me_rated') else [],
            'hsg_rated': copy.deepcopy(self.hsg_rated) if hasattr(self, 'hsg_rated') else [],
            'load_hist': copy.deepcopy(self.load_hist) if hasattr(self, 'load_hist') else [],
            'fuel_rate_me': copy.deepcopy(self.fuel_rate_me) if hasattr(self, 'fuel_rate_me') else [],
            'fuel_rate_hsg': copy.deepcopy(self.fuel_rate_hsg) if hasattr(self, 'fuel_rate_hsg') else [],
            'fuel_me': copy.deepcopy(self.fuel_me) if hasattr(self, 'fuel_me') else [],
            'fuel_hsg': copy.deepcopy(self.fuel_hsg) if hasattr(self, 'fuel_hsg') else [],
            'fuel': copy.deepcopy(self.fuel) if hasattr(self, 'fuel') else [],
            'fuel_rate': copy.deepcopy(self.fuel_rate) if hasattr(self, 'fuel_rate') else [],
            'load_perc_me': copy.deepcopy(self.load_perc_me) if hasattr(self, 'load_perc_me') else [],
            'load_perc_hsg': copy.deepcopy(self.load_perc_hsg) if hasattr(self, 'load_perc_hsg') else [],
            'power_total': copy.deepcopy(self.power_total) if hasattr(self, 'power_total') else [],
            'power_prop': copy.deepcopy(self.power_prop) if hasattr(self, 'power_prop') else [],
        }
        
    def reset(self):
        ''' 
        Reset machinery internal states using the recorded snapshot. 
        '''
        for key, value in self._initial_state.items():
            setattr(self, key, copy.deepcopy(value))

        # Reset integrator explicitly
        self.int = EulerInt()
        self.int.set_dt(self.int.dt)  # reuse current timestep


    
###################################################################################################################
######################## DESCENDANT CLASS BASED ON PARENT CLASS "BaseMachineryModel" ##############################
###################################################################################################################

class ShipMachineryModel(BaseMachineryModel):
    def __init__(self,
                 machinery_config: MachinerySystemConfiguration,
                 initial_propeller_shaft_speed_rad_per_sec: float,
                 initial_propeller_shaft_acc_rad_per_sec2: float,
                 time_step: float,
                 ):
        super().__init__(
            fuel_coeffs_for_main_engine=machinery_config.specific_fuel_consumption_coefficients_me,
            fuel_coeffs_for_diesel_gen=machinery_config.specific_fuel_consumption_coefficients_dg,
            rudder_config=RudderConfiguration(
                rudder_angle_to_yaw_force_coefficient=machinery_config.rudder_angle_to_yaw_force_coefficient,
                rudder_angle_to_sway_force_coefficient=machinery_config.rudder_angle_to_sway_force_coefficient,
                max_rudder_angle_degrees=machinery_config.max_rudder_angle_degrees
            ),
            machinery_modes=machinery_config.machinery_modes,
            hotel_load=machinery_config.hotel_load,
            operating_mode=machinery_config.machinery_operating_mode,
            time_step=time_step)
        self.w_rated_me = machinery_config.rated_speed_main_engine_rpm * np.pi / 30
        self.d_me = machinery_config.linear_friction_main_engine
        self.d_hsg = machinery_config.linear_friction_hybrid_shaft_generator
        self.r_me = machinery_config.gear_ratio_between_main_engine_and_propeller
        self.r_hsg = machinery_config.gear_ratio_between_hybrid_shaft_generator_and_propeller
        self.jp = machinery_config.propeller_inertia
        self.kp = machinery_config.propeller_speed_to_torque_coefficient
        self.dp = machinery_config.propeller_diameter
        self.kt = machinery_config.propeller_speed_to_thrust_force_coefficient
        self.shaft_speed_max = 1.1 * self.w_rated_me * self.r_me

        self.omega = initial_propeller_shaft_speed_rad_per_sec
        self.d_omega = initial_propeller_shaft_acc_rad_per_sec2

        # OPERATING MODES NAME TAG
        self.operating_mode = machinery_config.machinery_modes.list_of_modes[machinery_config.machinery_operating_mode].name_tag
        
        # Set up integration
        self.time_step = time_step
        self.int = EulerInt()  # Instantiate the Euler integrator
        self.int.set_dt(self.time_step)

        self.specific_fuel_coeffs_for_main_engine = FuelConsumptionCoefficients(a=128.89, b=-168.93, c=246.76)
        self.specific_fuel_coeffs_for_dg = FuelConsumptionCoefficients(a=180.71, b=-289.90, c=324.90)
        self.fuel_cons_me = 0.0  # Initial fuel cons for ME
        self.fuel_cons_electrical = 0.0  # Initial fuel cons for HSG
        self.fuel_cons = 0.0  # Initial total fuel cons
        self.power_me = []  # Array for storing ME power cons. data
        self.power_hsg = []  # Array for storing HSG power cons. data
        self.me_rated = []  # Array for storing ME rated power data
        self.hsg_rated = []  # Array for storing HSG rated power data
        self.load_hist = []  # Array for storing load percentage history
        self.fuel_rate_me = []  # Array for storing ME fuel cons. rate
        self.fuel_rate_hsg = []  # Array for storing HSG fuel cons. rate
        self.fuel_me = []  # Array for storing ME fuel cons.
        self.fuel_hsg = []  # Array for storing HSG fuel cons.
        self.fuel = []  # Array for storing total fuel cons
        self.fuel_rate = []
        self.load_perc_me = []
        self.load_perc_hsg = []
        self.power_total = []
        self.power_prop = []
        
        # Record of the initial parameters after everything is initialized
        super().record_initial_parameters()
        self.record_initial_parameters()

    def shaft_eq(self, torque_main_engine, torque_hsg):
        ''' Updates the time differential of the shaft speed
            equation.
        '''
        eq_me = (torque_main_engine - self.d_me * self.omega) / self.r_me
        eq_hsg = (torque_hsg - self.d_hsg * self.omega) / self.r_hsg
        self.d_omega = (eq_me + eq_hsg - self.kp * self.omega ** 2) / self.jp
        
        # print('eq_me :', eq_me)
        # print('eq_hsg:', eq_hsg)
        # print('top   :', eq_me + eq_hsg - self.kp * self.omega ** 2)
        # print('rad/s2:', self.d_omega)

    def thrust(self):
        ''' Updates the thrust force based on the shaft speed (self.omega)
        '''
        return self.dp ** 4 * self.kt * self.omega * abs(self.omega)

    def main_engine_torque(self, load_perc):
        ''' Returns the torque of the main engine as a
            function of the load percentage parameter
        '''
        if load_perc is None:
            return 0
        return min(load_perc * self.mode.available_propulsion_power_main_engine / (self.omega + 0.1),
                       self.mode.available_propulsion_power_main_engine / 5 * np.pi / 30)

    def hsg_torque(self, load_perc):
        ''' Returns the torque of the HSG as a
            function of the load percentage parameter
        '''
        if load_perc is None:
            return 0
        return min(load_perc * self.mode.available_propulsion_power_electrical / (self.omega + 0.1),
                   self.mode.available_propulsion_power_electrical / 5 * np.pi / 30)

    def integrate_differentials(self):
        ''' Integrates the differential equation one time step ahead
        '''
        # print('omega  :', self.omega)
        # print('d_omega:', self.d_omega)
        self.omega = self.int.integrate(x=self.omega, dx=self.d_omega)

    def update_shaft_equation(self, load_percentage):
        if load_percentage is None: load_percentage = 0.0
        self.shaft_eq(
            torque_main_engine=self.main_engine_torque(load_perc=load_percentage),
            torque_hsg=self.hsg_torque(load_perc=load_percentage)
        )
        
    def record_initial_parameters(self):
        '''
        Internal method to take a record of internal attributes after __init__().
        This record will be used to reset the model later without re-instantiation.
        '''
        # Store BaseMachineryModel parameters
        super().record_initial_parameters()
        
        # Also store the current class's parameters
        self._initial_parameters = {
            key: copy.deepcopy(self.__dict__[key])
            for key in ['w_rated_me', 'd_me', 'd_hsg', 'r_me', 'r_hsg',
                        'jp', 'kp', 'dp', 'kt', 'shaft_speed_max',
                        'omega', 'd_omega', 'time_step', 
                        'specific_fuel_coeffs_for_main_engine',
                        'specific_fuel_coeffs_for_dg',
                        'fuel_cons_me', 'fuel_cons_electrical', 'fuel_cons',
                        'power_me', 'power_hsg', 'me_rated', 'hsg_rated',
                        'load_hist', 'fuel_rate_me', 'fuel_rate_hsg',
                        'fuel_me', 'fuel_hsg', 'fuel', 'fuel_rate',
                        'load_perc_me', 'load_perc_hsg', 'power_total', 'power_prop']
            }

    def reset(self):
        '''
        Resets the internal state of the ship to its initial record.
        Should be called at the beginning of each episode.
        '''
        # Reset super class (Base Machinery Model) NOTE: RESET THE BASE CLASS FIRST
        super().reset()
        
        for key, value in self._initial_parameters.items():
            setattr(self, key, copy.deepcopy(value))

        # Re-create integrator and drawer
        self.int = EulerInt()
        self.int.set_dt(self.time_step)