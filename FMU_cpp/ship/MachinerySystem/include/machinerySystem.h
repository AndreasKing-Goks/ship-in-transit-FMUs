#ifndef MACHINERY_SYSTEM_H
#define MACHINERY_SYSTEM_H
#include "modelName.h"
#include "systemBaseWithSolvers.h"

#include <string>

// Machinery System C++ FMU implementation.
// Machinery system and machinery system operating mode switching.
// Port of FMU_scripts/ship/machinery_system_fmu.py
class MODEL_IDENTIFIER : public SystemBaseWithSolvers
{
public:
    MODEL_IDENTIFIER(std::shared_ptr<LoggerBase> _logger = {}, std::string _location = "");

    // Runs do_step() once per communication step
    void DoStep(const double globalTime) override;

private:
    void do_step(double current_time, double step_size);
    double shaft_eq(double torque_main_engine, double torque_hsg);
    double get_thrust_force(double omega);
    double main_engine_torque(double load_perc);
    double hsg_torque(double load_perc);
    double update_shaft_equation(double load_perc);
    void distribute_load(double load_perc, double& load_main_engine, double& load_electrical,
                         double& load_percentage_main_engine, double& load_percentage_electrical);
    double spec_fuel_cons(double load_perc, double coeff_a, double coeff_b, double coeff_c);
    void get_fuel_consumption(double load_perc, double step_size, double& rate_me, double& rate_hsg,
                              double& fuel_consumption_me, double& fuel_consumption_hsg, double& fuel_consumption);

    // Parameters
    // Machinery System Configuration
    double hotel_load;
    double rated_speed_main_engine_rpm;
    double linear_friction_main_engine;
    double linear_friction_hybrid_shaft_generator;
    double gear_ratio_between_main_engine_and_propeller;
    double gear_ratio_between_hybrid_shaft_generator_and_propeller;
    double propeller_inertia;
    double propeller_speed_to_torque_coefficient;
    double propeller_diameter;
    double propeller_speed_to_thrust_force_coefficient;
    double specific_fuel_consumption_coefficients_me_a_coeff;
    double specific_fuel_consumption_coefficients_me_b_coeff;
    double specific_fuel_consumption_coefficients_me_c_coeff;
    double specific_fuel_consumption_coefficients_dg_a_coeff;
    double specific_fuel_consumption_coefficients_dg_b_coeff;
    double specific_fuel_consumption_coefficients_dg_c_coeff;

    double omega;
    double d_omega;

    // Engine Parameters
    double main_engine_capacity_spec;
    double diesel_gen_capacity_spec;
    int mso_mode;  // 0:MEC, 1:PTO, 2:PTI

    // Input Variables
    double load_perc;

    // Output Variables
    double thrust_force;
    double shaft_speed_rpm;
    double cmd_load_fraction_me;
    double cmd_load_fraction_hsg;
    double power_me;
    double available_power_me;
    double power_electrical;
    double available_power_electrical;
    double power;
    double propulsion_power;
    double fuel_rate_me;
    double fuel_rate_hsg;
    double fuel_rate;
    double fuel_consumption_me;
    double fuel_consumption_hsg;
    double fuel_consumption;
    double motor_torque;
    double hybrid_shaft_generator_torque;

    // Internal Variables
    double main_engine_capacity = 0.0;
    double electrical_capacity = 0.0;
    std::string shaft_generator_state = "OFF";  // default
    double available_propulsion_power = 0.0;
    double available_propulsion_power_main_engine = 0.0;
    double available_propulsion_power_electrical = 0.0;
    double fuel_consumption_me_temp = 0.0;
    double fuel_consumption_hsg_temp = 0.0;
    double fuel_consumption_temp = 0.0;
};

#endif
