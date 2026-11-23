#include "modelName.h"
#include "systemBase.h"
#include SYSTEM_HEADER
#include "guid.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numbers>
#include <stdexcept>

MODEL_IDENTIFIER::MODEL_IDENTIFIER(std::shared_ptr<LoggerBase> _logger, std::string _location)
    : SystemBaseWithSolvers(_logger, _location)
{
    author      = "Andreas R.G. Sitorus, Etienne Purcell";
    description = "Ship Machinery System C++ FMU Implementation";
    version     = MODEL_VERSION;
    name        = MODELNAME;
    identifier  = MODEL_IDENTIFIER_STR;
    guid        = fmuGUID;

    // No built-in solver: do_step() does the work
    solverType = "None";
    printInfo  = false;

    // Registration
    // Machinery System Configuration
    RegisterReal(hotel_load, "hotel_load", 0.0);
    RegisterReal(rated_speed_main_engine_rpm, "rated_speed_main_engine_rpm", 0.0);
    RegisterReal(linear_friction_main_engine, "linear_friction_main_engine", 0.0);
    RegisterReal(linear_friction_hybrid_shaft_generator, "linear_friction_hybrid_shaft_generator", 0.0);
    RegisterReal(gear_ratio_between_main_engine_and_propeller, "gear_ratio_between_main_engine_and_propeller", 0.0);
    RegisterReal(gear_ratio_between_hybrid_shaft_generator_and_propeller, "gear_ratio_between_hybrid_shaft_generator_and_propeller", 0.0);
    RegisterReal(propeller_inertia, "propeller_inertia", 0.0);
    RegisterReal(propeller_speed_to_torque_coefficient, "propeller_speed_to_torque_coefficient", 0.0);
    RegisterReal(propeller_diameter, "propeller_diameter", 0.0);
    RegisterReal(propeller_speed_to_thrust_force_coefficient, "propeller_speed_to_thrust_force_coefficient", 0.0);
    RegisterReal(specific_fuel_consumption_coefficients_me_a_coeff, "specific_fuel_consumption_coefficients_me_a_coeff", 0.0);
    RegisterReal(specific_fuel_consumption_coefficients_me_b_coeff, "specific_fuel_consumption_coefficients_me_b_coeff", 0.0);
    RegisterReal(specific_fuel_consumption_coefficients_me_c_coeff, "specific_fuel_consumption_coefficients_me_c_coeff", 0.0);
    RegisterReal(specific_fuel_consumption_coefficients_dg_a_coeff, "specific_fuel_consumption_coefficients_dg_a_coeff", 0.0);
    RegisterReal(specific_fuel_consumption_coefficients_dg_b_coeff, "specific_fuel_consumption_coefficients_dg_b_coeff", 0.0);
    RegisterReal(specific_fuel_consumption_coefficients_dg_c_coeff, "specific_fuel_consumption_coefficients_dg_c_coeff", 0.0);

    // omega and d_omega are tunable parameters in the Python FMU, but used as state
    RegisterReal(omega,   "omega",   0.0, ScalarVariable::Type::TunableParameter);
    RegisterReal(d_omega, "d_omega", 0.0, ScalarVariable::Type::TunableParameter);

    // Engine Parameters
    RegisterReal(main_engine_capacity_spec, "main_engine_capacity_spec", 0.0);
    RegisterReal(diesel_gen_capacity_spec,  "diesel_gen_capacity_spec",  0.0);
    RegisterInteger(mso_mode, "mso_mode", 0, ScalarVariable::Type::TunableParameter);

    // fmicpp 0.5 registers Type::TunableParameter as fixed, so set it again here
    variability["omega"]    = ScalarVariable::Variability::Tunable;
    variability["d_omega"]  = ScalarVariable::Variability::Tunable;
    variability["mso_mode"] = ScalarVariable::Variability::Tunable;

    // Input Variables
    RegisterInput(load_perc, "load_perc", 0.0);

    // Output Variables
    RegisterOutput(thrust_force, "thrust_force", 0.0);
    RegisterOutput(shaft_speed_rpm, "shaft_speed_rpm", 0.0);
    RegisterOutput(cmd_load_fraction_me, "cmd_load_fraction_me", 0.0);
    RegisterOutput(cmd_load_fraction_hsg, "cmd_load_fraction_hsg", 0.0);
    RegisterOutput(power_me, "power_me", 0.0);
    RegisterOutput(available_power_me, "available_power_me", 0.0);
    RegisterOutput(power_electrical, "power_electrical", 0.0);
    RegisterOutput(available_power_electrical, "available_power_electrical", 0.0);
    RegisterOutput(power, "power", 0.0);
    RegisterOutput(propulsion_power, "propulsion_power", 0.0);
    RegisterOutput(fuel_rate_me, "fuel_rate_me", 0.0);
    RegisterOutput(fuel_rate_hsg, "fuel_rate_hsg", 0.0);
    RegisterOutput(fuel_rate, "fuel_rate", 0.0);
    RegisterOutput(fuel_consumption_me, "fuel_consumption_me", 0.0);
    RegisterOutput(fuel_consumption_hsg, "fuel_consumption_hsg", 0.0);
    RegisterOutput(fuel_consumption, "fuel_consumption", 0.0);
    RegisterOutput(motor_torque, "motor_torque", 0.0);
    RegisterOutput(hybrid_shaft_generator_torque, "hybrid_shaft_generator_torque", 0.0);

    // Fuel consumption totals, kept between steps
    RegisterInternal(fuel_consumption_me_temp,  "fuel_consumption_me_temp",  0.0);
    RegisterInternal(fuel_consumption_hsg_temp, "fuel_consumption_hsg_temp", 0.0);
    RegisterInternal(fuel_consumption_temp,     "fuel_consumption_temp",     0.0);
}

// Division that fails like Python (ZeroDivisionError) instead of giving inf or nan
static double div(double num, double den)
{
    if (den == 0.0)
    {
        throw std::domain_error("float division by zero");
    }
    return num / den;
}

// Updates the time differential of the shaft speed equation.
double MODEL_IDENTIFIER::shaft_eq(double torque_main_engine, double torque_hsg)
{
    double eq_me   = div(torque_main_engine - linear_friction_main_engine * omega, gear_ratio_between_main_engine_and_propeller);
    double eq_hsg  = div(torque_hsg - linear_friction_hybrid_shaft_generator * omega, gear_ratio_between_hybrid_shaft_generator_and_propeller);
    double d_omega_new = div(eq_me + eq_hsg - propeller_speed_to_torque_coefficient * std::pow(omega, 2), propeller_inertia);
    return d_omega_new;
}

// Thrust force based on the shaft speed (omega)
double MODEL_IDENTIFIER::get_thrust_force(double omega)
{
    return std::pow(propeller_diameter, 4) * propeller_speed_to_thrust_force_coefficient * omega * std::abs(omega);
}

// Torque of the main engine as a function of the load percentage parameter
double MODEL_IDENTIFIER::main_engine_torque(double load_perc)
{
    return std::min(div(load_perc * available_propulsion_power_main_engine, omega + 0.1),
                    available_propulsion_power_main_engine / 5 * std::numbers::pi / 30);
}

// Torque of the HSG as a function of the load percentage parameter
double MODEL_IDENTIFIER::hsg_torque(double load_perc)
{
    return std::min(div(load_perc * available_propulsion_power_electrical, omega + 0.1),
                    available_propulsion_power_electrical / 5 * std::numbers::pi / 30);
}

double MODEL_IDENTIFIER::update_shaft_equation(double load_perc)
{
    return shaft_eq(main_engine_torque(load_perc), hsg_torque(load_perc));
}

void MODEL_IDENTIFIER::distribute_load(double load_perc, double& load_main_engine, double& load_electrical,
                                       double& load_percentage_main_engine, double& load_percentage_electrical)
{
    double total_load_propulsion = load_perc * available_propulsion_power;
    if (shaft_generator_state == "MOTOR")
    {
        load_main_engine = std::min(total_load_propulsion, main_engine_capacity);
        load_electrical = total_load_propulsion + hotel_load - load_main_engine;
        load_percentage_electrical = div(load_electrical, electrical_capacity);
        if (main_engine_capacity == 0)
        {
            load_percentage_main_engine = 0;
        }
        else
        {
            load_percentage_main_engine = load_main_engine / main_engine_capacity;
        }
    }
    else if (shaft_generator_state == "GEN")
    {
        // Here the rule is that electrical handles hotel as far as possible
        load_electrical = std::min(hotel_load, electrical_capacity);
        load_main_engine = total_load_propulsion + hotel_load - load_electrical;
        load_percentage_main_engine = div(load_main_engine, main_engine_capacity);
        if (electrical_capacity == 0)
        {
            load_percentage_electrical = 0;
        }
        else
        {
            load_percentage_electrical = load_electrical / electrical_capacity;
        }
    }
    else
    {
        load_main_engine = total_load_propulsion;
        load_electrical = hotel_load;
        load_percentage_main_engine = div(load_main_engine, main_engine_capacity);
        load_percentage_electrical = div(load_electrical, electrical_capacity);
    }
}

// Calculate fuel consumption rate for engine.
double MODEL_IDENTIFIER::spec_fuel_cons(double load_perc, double coeff_a, double coeff_b, double coeff_c)
{
    double rate = coeff_a * std::pow(load_perc, 2) + coeff_b * load_perc + coeff_c;
    return rate / 3.6e9;
}

// Fuel consumption rates and total fuel consumption
void MODEL_IDENTIFIER::get_fuel_consumption(double load_perc, double step_size, double& rate_me, double& rate_hsg,
                                            double& fuel_consumption_me, double& fuel_consumption_hsg, double& fuel_consumption)
{
    double load_main_engine, load_electrical, load_percentage_main_engine, load_percentage_electrical;
    distribute_load(load_perc, load_main_engine, load_electrical, load_percentage_main_engine, load_percentage_electrical);

    if (load_main_engine == 0.0)
    {
        rate_me = 0.0;
    }
    else
    {
        rate_me = load_main_engine * spec_fuel_cons(load_percentage_main_engine,
                                                    specific_fuel_consumption_coefficients_me_a_coeff,
                                                    specific_fuel_consumption_coefficients_me_b_coeff,
                                                    specific_fuel_consumption_coefficients_me_c_coeff);
    }

    if (load_percentage_electrical == 0.0)
    {
        rate_hsg = 0.0;
    }
    else
    {
        rate_hsg = load_electrical * spec_fuel_cons(load_percentage_electrical,
                                                    specific_fuel_consumption_coefficients_dg_a_coeff,
                                                    specific_fuel_consumption_coefficients_dg_b_coeff,
                                                    specific_fuel_consumption_coefficients_dg_c_coeff);
    }

    fuel_consumption_me  = fuel_consumption_me_temp + rate_me * step_size;
    fuel_consumption_hsg = fuel_consumption_hsg_temp + rate_hsg * step_size;
    fuel_consumption     = fuel_consumption_temp + (rate_me + rate_hsg) * step_size;
}

void MODEL_IDENTIFIER::do_step(double current_time, double step_size)
{
    try
    {
        // TREAT MACHINERY MODE AS AN INPUT
        // MEC Mode
        if (mso_mode == 0)
        {
            main_engine_capacity  = main_engine_capacity_spec;
            electrical_capacity   = diesel_gen_capacity_spec;       // Use 1 generator
            shaft_generator_state = "OFF";
        }
        // PTO Mode
        else if (mso_mode == 1)
        {
            main_engine_capacity  = main_engine_capacity_spec;
            electrical_capacity   = 0.0;
            shaft_generator_state = "GEN";
        }
        // PTI Mode
        else if (mso_mode == 2)
        {
            main_engine_capacity  = 0.0;
            electrical_capacity   = 2 * diesel_gen_capacity_spec;   // Use 2 generators
            shaft_generator_state = "MOTOR";
        }

        // Update Available Propulsion Power
        if (shaft_generator_state == "MOTOR")
        {
            available_propulsion_power = main_engine_capacity + electrical_capacity - hotel_load;
            available_propulsion_power_main_engine = main_engine_capacity;
            available_propulsion_power_electrical = electrical_capacity - hotel_load;
        }
        else if (shaft_generator_state == "GEN")
        {
            available_propulsion_power = main_engine_capacity - hotel_load;
            available_propulsion_power_main_engine = main_engine_capacity - hotel_load;
            available_propulsion_power_electrical = 0;
        }
        else if (shaft_generator_state == "OFF")
        {
            available_propulsion_power = main_engine_capacity;
            available_propulsion_power_main_engine = main_engine_capacity;
            available_propulsion_power_electrical = 0;
        }

        // Max shaft speed
        double shaft_speed_max = 1.1 * (rated_speed_main_engine_rpm * std::numbers::pi / 30) * gear_ratio_between_main_engine_and_propeller;

        // Get the shaft speed using input
        d_omega = update_shaft_equation(load_perc);
        omega   = std::min(omega + d_omega * step_size, shaft_speed_max);   // Integration

        // Get the thrust
        thrust_force = get_thrust_force(omega);

        // Compute the shaft speed in rpm
        shaft_speed_rpm = omega * 30 / std::numbers::pi;

        // Get the commanded load fraction
        double load_main_engine, load_electrical, load_percentage_main_engine, load_percentage_electrical;
        distribute_load(load_perc, load_main_engine, load_electrical, load_percentage_main_engine, load_percentage_electrical);
        cmd_load_fraction_me       = load_percentage_main_engine;
        cmd_load_fraction_hsg      = load_percentage_electrical;
        power_me                   = load_main_engine / 1000;
        available_power_me         = main_engine_capacity / 1000;
        power_electrical           = load_electrical / 1000;
        available_power_electrical = electrical_capacity / 1000;
        power                      = (load_main_engine + load_electrical) / 1000;
        propulsion_power           = available_propulsion_power;

        // Get fuel consumption
        double rate_me, rate_hsg, fuel_consumption_me_new, fuel_consumption_hsg_new, fuel_consumption_new;
        get_fuel_consumption(load_perc, step_size, rate_me, rate_hsg,
                             fuel_consumption_me_new, fuel_consumption_hsg_new, fuel_consumption_new);
        fuel_rate_me                  = rate_me;
        fuel_rate_hsg                 = rate_hsg;
        fuel_rate                     = rate_me + rate_hsg;
        fuel_consumption_me           = fuel_consumption_me_new;
        fuel_consumption_hsg          = fuel_consumption_hsg_new;
        fuel_consumption              = fuel_consumption_new;
        // Keep the totals for the next step
        fuel_consumption_me_temp      = fuel_consumption_me_new;
        fuel_consumption_hsg_temp     = fuel_consumption_hsg_new;
        fuel_consumption_temp         = fuel_consumption_new;
        motor_torque                  = main_engine_torque(load_perc);
        hybrid_shaft_generator_torque = hsg_torque(load_perc);
    }
    catch (const std::exception& e)
    {
        // IMPORTANT: do not crash host
        std::cout << "[MachinerySystem] ERROR t=" << current_time << " dt=" << step_size << ": " << e.what() << std::endl;

        thrust_force                  = 0.0;
        shaft_speed_rpm               = 0.0;
        cmd_load_fraction_me          = 0.0;
        cmd_load_fraction_hsg         = 0.0;
        power_me                      = 0.0;
        available_power_me            = 0.0;
        power_electrical              = 0.0;
        available_power_electrical    = 0.0;
        power                         = 0.0;
        propulsion_power              = 0.0;
        fuel_rate_me                  = 0.0;
        fuel_rate_hsg                 = 0.0;
        fuel_rate                     = 0.0;
        fuel_consumption_me           = 0.0;
        fuel_consumption_hsg          = 0.0;
        fuel_consumption              = 0.0;
        motor_torque                  = 0.0;
        hybrid_shaft_generator_torque = 0.0;
    }
}

void MODEL_IDENTIFIER::DoStep(const double globalTime)
{
    do_step(time, globalTime - time);
    SystemBaseWithSolvers::DoStep(globalTime);
}
