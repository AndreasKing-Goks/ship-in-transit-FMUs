"""
Machinery System Python FMU implementation.
This FMU manages a machinery system and machinery system operating mode switching.

Authors : Andreas R.G. Sitorus
Date    : Januari 2026
"""

from pythonfmu import Fmi2Causality, Fmi2Slave, Fmi2Variability, Real, Integer, Boolean, String
import numpy as np

class MachinerySystem(Fmi2Slave):
    
    author = "Andreas R.G. Sitorus"
    description = "Ship Machinery System Python FMU Implementation"
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        
        ## Parameters
        # Machinery System Configuration
        self.hotel_load                                                 = 0.0
        self.rated_speed_main_engine_rpm                                = 0.0
        self.linear_friction_main_engine                                = 0.0
        self.linear_friction_hybrid_shaft_generator                     = 0.0
        self.gear_ratio_between_main_engine_and_propeller               = 0.0
        self.gear_ratio_between_hybrid_shaft_generator_and_propeller    = 0.0
        self.propeller_inertia                                          = 0.0
        self.propeller_speed_to_torque_coefficient                      = 0.0
        self.propeller_diameter                                         = 0.0
        self.propeller_speed_to_thrust_force_coefficient                = 0.0
        self.specific_fuel_consumption_coefficients_me_a_coeff          = 0.0
        self.specific_fuel_consumption_coefficients_me_b_coeff          = 0.0
        self.specific_fuel_consumption_coefficients_me_c_coeff          = 0.0
        self.specific_fuel_consumption_coefficients_dg_a_coeff          = 0.0
        self.specific_fuel_consumption_coefficients_dg_b_coeff          = 0.0
        self.specific_fuel_consumption_coefficients_dg_c_coeff          = 0.0
        
        self.omega                                                      = 0.0
        self.d_omega                                                    = 0.0
        
        # Engine Parameters
        self.main_engine_capacity_spec                                  = 0.0
        self.diesel_gen_capacity_spec                                   = 0.0
        self.mso_mode                                                   = 0 # 0:MEC, 1:PTO, 2:PTI
        
        ## Input Variables
        self.load_perc                                                  = 0.0
        
        ## Output Variables
        self.thrust_force                                               = 0.0
        self.shaft_speed_rpm                                            = 0.0
        self.cmd_load_fraction_me                                       = 0.0
        self.cmd_load_fraction_hsg                                      = 0.0
        self.power_me                                                   = 0.0
        self.available_power_me                                         = 0.0
        self.power_electrical                                           = 0.0
        self.available_power_electrical                                 = 0.0
        self.power                                                      = 0.0
        self.propulsion_power                                           = 0.0
        self.fuel_rate_me                                               = 0.0
        self.fuel_rate_hsg                                              = 0.0
        self.fuel_rate                                                  = 0.0
        self.fuel_consumption_me                                        = 0.0
        self.fuel_consumption_hsg                                       = 0.0
        self.fuel_consumption                                           = 0.0
        self.motor_torque                                               = 0.0
        self.hybrid_shaft_generator_torque                              = 0.0
        
        ## Internal Variables
        self.main_engine_capacity                                       = self.main_engine_capacity_spec
        self.electrical_capacity                                        = 2 * self.diesel_gen_capacity_spec
        self.shaft_generator_state                                      = 'OFF' # default
        self.available_propulsion_power                                 = 0.0
        self.available_propulsion_power_main_engine                     = 0.0
        self.available_propulsion_power_electrical                      = 0.0
        self.load_on_main_engine                                        = 0.0
        self.load_on_electrical                                         = 0.0
        self.load_percentage_on_main_engine                             = 0.0
        self.load_percentage_electrical                                 = 0.0
        self.fuel_consumption_me_temp                                   = 0.0
        self.fuel_consumption_hsg_temp                                  = 0.0
        self.fuel_consumption_temp                                      = 0.0
            
        ## Registration
        # Rudder Parameters        
        # Machinery System Configuration
        self.register_variable(Real("hotel_load", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        self.register_variable(Real("rated_speed_main_engine_rpm", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        self.register_variable(Real("linear_friction_main_engine", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        self.register_variable(Real("linear_friction_hybrid_shaft_generator", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        self.register_variable(Real("gear_ratio_between_main_engine_and_propeller", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        self.register_variable(Real("gear_ratio_between_hybrid_shaft_generator_and_propeller", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        self.register_variable(Real("propeller_inertia", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        self.register_variable(Real("propeller_speed_to_torque_coefficient", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        self.register_variable(Real("propeller_diameter", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        self.register_variable(Real("propeller_speed_to_thrust_force_coefficient", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        self.register_variable(Real("specific_fuel_consumption_coefficients_me_a_coeff", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        self.register_variable(Real("specific_fuel_consumption_coefficients_me_b_coeff", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        self.register_variable(Real("specific_fuel_consumption_coefficients_me_c_coeff", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        self.register_variable(Real("specific_fuel_consumption_coefficients_dg_a_coeff", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        self.register_variable(Real("specific_fuel_consumption_coefficients_dg_b_coeff", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        self.register_variable(Real("specific_fuel_consumption_coefficients_dg_c_coeff", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        
        self.register_variable(Real("omega", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
        self.register_variable(Real("d_omega", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
        
        # Engine Parameters
        self.register_variable(Real("main_engine_capacity_spec", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        self.register_variable(Real("diesel_gen_capacity_spec", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        self.register_variable(Integer("mso_mode", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
            
        # Input Variables
        self.register_variable(Real("load_perc", causality=Fmi2Causality.input))
        
        # Output Variables
        self.register_variable(Real("thrust_force", causality=Fmi2Causality.output))
        self.register_variable(Real("shaft_speed_rpm", causality=Fmi2Causality.output))
        self.register_variable(Real("cmd_load_fraction_me", causality=Fmi2Causality.output))
        self.register_variable(Real("cmd_load_fraction_hsg", causality=Fmi2Causality.output))
        self.register_variable(Real("power_me", causality=Fmi2Causality.output))
        self.register_variable(Real("available_power_me", causality=Fmi2Causality.output))
        self.register_variable(Real("power_electrical", causality=Fmi2Causality.output))
        self.register_variable(Real("available_power_electrical", causality=Fmi2Causality.output))
        self.register_variable(Real("power", causality=Fmi2Causality.output))
        self.register_variable(Real("propulsion_power", causality=Fmi2Causality.output))
        self.register_variable(Real("fuel_rate_me", causality=Fmi2Causality.output))
        self.register_variable(Real("fuel_rate_hsg", causality=Fmi2Causality.output))
        self.register_variable(Real("fuel_rate", causality=Fmi2Causality.output))
        self.register_variable(Real("fuel_consumption_me", causality=Fmi2Causality.output))
        self.register_variable(Real("fuel_consumption_hsg", causality=Fmi2Causality.output))
        self.register_variable(Real("fuel_consumption", causality=Fmi2Causality.output))
        self.register_variable(Real("motor_torque", causality=Fmi2Causality.output))
        self.register_variable(Real("hybrid_shaft_generator_torque", causality=Fmi2Causality.output))
        
        
    def shaft_eq(self, torque_main_engine, torque_hsg):
        ''' Updates the time differential of the shaft speed
            equation.
        '''
        eq_me = (torque_main_engine - self.linear_friction_main_engine * self.omega) / self.gear_ratio_between_main_engine_and_propeller
        eq_hsg = (torque_hsg - self.linear_friction_hybrid_shaft_generator * self.omega) / self.gear_ratio_between_hybrid_shaft_generator_and_propeller
        d_omega = (eq_me + eq_hsg - self.propeller_speed_to_torque_coefficient * self.omega ** 2) / self.propeller_inertia
        return d_omega


    def get_thrust_force(self, omega):
        ''' Updates the thrust force based on the shaft speed (self.omega)
        '''
        return self.propeller_diameter ** 4 * self.propeller_speed_to_thrust_force_coefficient * omega * abs(omega)


    def main_engine_torque(self, load_perc):
        ''' Returns the torque of the main engine as a
            function of the load percentage parameter
        '''
        if load_perc is None:
            return 0
        return min(load_perc * self.available_propulsion_power_main_engine / (self.omega + 0.1),
                       self.available_propulsion_power_main_engine / 5 * np.pi / 30)


    def hsg_torque(self, load_perc):
        ''' Returns the torque of the HSG as a
            function of the load percentage parameter
        '''
        if load_perc is None:
            return 0
        return min(load_perc * self.available_propulsion_power_electrical / (self.omega + 0.1),
                   self.available_propulsion_power_electrical / 5 * np.pi / 30)
        

    def update_shaft_equation(self, load_perc):
        if load_perc is None: load_perc = 0.0
        d_omega = self.shaft_eq(
                        torque_main_engine=self.main_engine_torque(load_perc=load_perc),
                        torque_hsg=self.hsg_torque(load_perc=load_perc)
                    )
        return d_omega
    
    
    def distribute_load(self, load_perc):
        total_load_propulsion = load_perc * self.available_propulsion_power
        if self.shaft_generator_state == 'MOTOR':
            load_main_engine = min(total_load_propulsion, self.main_engine_capacity)
            load_electrical = total_load_propulsion + self.hotel_load - load_main_engine
            load_percentage_electrical = load_electrical / self.electrical_capacity
            if self.main_engine_capacity == 0:
                load_percentage_main_engine = 0
            else:
                load_percentage_main_engine = load_main_engine / self.main_engine_capacity
        elif self.shaft_generator_state == 'GEN':
            # Here the rule is that electrical handles hotel as far as possible
            load_electrical = min(self.hotel_load, self.electrical_capacity)
            load_main_engine = total_load_propulsion + self.hotel_load - load_electrical
            load_percentage_main_engine = load_main_engine / self.main_engine_capacity
            if self.electrical_capacity == 0:
                load_percentage_electrical = 0
            else:
                load_percentage_electrical = load_electrical / self.electrical_capacity
        else: 
            load_main_engine = total_load_propulsion
            load_electrical = self.hotel_load
            load_percentage_main_engine = load_main_engine / self.main_engine_capacity
            load_percentage_electrical = load_electrical / self.electrical_capacity

        return load_main_engine, load_electrical, load_percentage_main_engine, load_percentage_electrical
    
    
    def spec_fuel_cons(self, load_perc, coeff_a, coeff_b, coeff_c):
        """ Calculate fuel consumption rate for engine.
        """
        rate = coeff_a * load_perc ** 2 + coeff_b * load_perc + coeff_c
        return rate / 3.6e9
    
    
    def get_fuel_consumption(self, load_perc, step_size  ):
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
        load_main_engine, load_electrical, load_percentage_main_engine, load_percentage_electrical = self.distribute_load(load_perc)
        
        if load_main_engine == 0.0:
            rate_me = 0.0
        else:
            rate_me = load_main_engine * self.spec_fuel_cons(load_perc=load_percentage_main_engine,
                                                             coeff_a=self.specific_fuel_consumption_coefficients_me_a_coeff,
                                                                        coeff_b=self.specific_fuel_consumption_coefficients_me_b_coeff,
                                                                        coeff_c=self.specific_fuel_consumption_coefficients_me_c_coeff)
            
        if load_percentage_electrical == 0.0:
            rate_hsg = 0.0
        else:
            rate_hsg = load_electrical * self.spec_fuel_cons(load_perc=load_percentage_electrical,
                                                             coeff_a=self.specific_fuel_consumption_coefficients_dg_a_coeff,
                                                             coeff_b=self.specific_fuel_consumption_coefficients_dg_b_coeff,
                                                             coeff_c=self.specific_fuel_consumption_coefficients_dg_c_coeff)
            
        fuel_consumption_me            = self.fuel_consumption_me_temp + rate_me * step_size
        fuel_consumption_hsg           = self.fuel_consumption_hsg_temp + rate_hsg * step_size
        fuel_consumption               = self.fuel_consumption_temp + (rate_me + rate_hsg) * step_size
        
        return rate_me, rate_hsg, fuel_consumption_me, fuel_consumption_hsg, fuel_consumption
            
        
    def do_step(self, current_time: float, step_size: float) -> bool:
        ## TREAT MACHINERY MODE AS AN INPUT
        # MEC Mode
        if self.mso_mode == 0:
            self.main_engine_capacity = self.main_engine_capacity_spec
            self.electrical_capacity  = self.diesel_gen_capacity_spec       # Use 1 generator
            self.shaft_generator_state = 'OFF'
        # PTO Mode
        elif self.mso_mode == 1:
            self.main_engine_capacity = self.main_engine_capacity_spec
            self.electrical_capacity  = 0.0
            self.shaft_generator_state = 'GEN'
        # PTI Mode
        elif self.mso_mode == 2:
            self.main_engine_capacity = 0.0
            self.electrical_capacity  = 2 * self.diesel_gen_capacity_spec   # Use 2 generators
            self.shaft_generator_state = 'MOTOR'
        
        # Update Available Propulsion Power
        if self.shaft_generator_state == 'MOTOR':
            self.available_propulsion_power = self.main_engine_capacity + self.electrical_capacity - self.hotel_load
            self.available_propulsion_power_main_engine = self.main_engine_capacity
            self.available_propulsion_power_electrical = self.electrical_capacity - self.hotel_load
        elif self.shaft_generator_state == 'GEN':
            self.available_propulsion_power = self.main_engine_capacity - self.hotel_load
            self.available_propulsion_power_main_engine = self.main_engine_capacity - self.hotel_load
            self.available_propulsion_power_electrical = 0
        elif self.shaft_generator_state == 'OFF':
            self.available_propulsion_power = self.main_engine_capacity
            self.available_propulsion_power_main_engine = self.main_engine_capacity
            self.available_propulsion_power_electrical = 0
        
        # Max shaft speed
        shaft_speed_max = 1.1 * (self.rated_speed_main_engine_rpm * np.pi / 30) * self.gear_ratio_between_main_engine_and_propeller
        
        # Get the shaft speed using input
        self.d_omega = self.update_shaft_equation(self.load_perc)   
        self.omega   = np.min([(self.omega + self.d_omega * step_size), shaft_speed_max])          # Integration
        
        # Get the thrust
        self.thrust_force = self.get_thrust_force(self.omega)
        
        # Compute the shaft speed in rpm
        self.shaft_speed_rpm = self.omega * 30 /np.pi
        
        # Get the commanded load fraction
        load_main_engine, load_electrical, load_percentage_main_engine, load_percentage_electrical = self.distribute_load(self.load_perc)
        self.cmd_load_fraction_me               = load_percentage_main_engine
        self.cmd_load_fraction_hsg              = load_percentage_electrical
        self.power_me                           = load_main_engine / 1000
        self.available_power_me                 = self.main_engine_capacity / 1000
        self.power_electrical                   = load_electrical / 1000
        self.available_power_electrical         = self.electrical_capacity / 1000
        self.power                              = (load_main_engine + load_electrical) / 1000
        self.propulsion_power                   = self.available_propulsion_power
        
        # Get fuel consumption
        rate_me, rate_hsg, fuel_consumption_me, fuel_consumption_hsg, fuel_consumption = self.get_fuel_consumption(self.load_perc, step_size=step_size)
        self.fuel_rate_me                       = rate_me
        self.fuel_rate_hsg                      = rate_hsg
        self.fuel_rate                          = rate_me + rate_hsg
        self.fuel_consumption_me                = fuel_consumption_me
        self.fuel_consumption_hsg               = fuel_consumption_hsg
        self.fuel_consumption                   = fuel_consumption
        self.motor_torque                       = self.main_engine_torque(self.load_perc)
        self.hybrid_shaft_generator_torque      = self.hsg_torque(self.load_perc)
        
        return True    