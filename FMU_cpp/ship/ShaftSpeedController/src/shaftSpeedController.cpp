#include "modelName.h"
#include "systemBase.h"
#include SYSTEM_HEADER
#include "guid.h"

#include <algorithm>
#include <iostream>

MODEL_IDENTIFIER::MODEL_IDENTIFIER(std::shared_ptr<LoggerBase> _logger, std::string _location)
    : SystemBaseWithSolvers(_logger, _location)
{
    author      = "Andreas R.G. Sitorus, Etienne Purcell";
    description = "PI-Based Shaft Speed Controller C++ FMU Implementation";
    version     = MODEL_VERSION;
    name        = MODELNAME;
    identifier  = MODEL_IDENTIFIER_STR;
    guid        = fmuGUID;

    // No built-in solver: do_step() does the work
    solverType = "None";
    printInfo  = false;

    // Registration
    // PI Parameters
    RegisterReal(kp, "kp", 0.0, ScalarVariable::Type::TunableParameter);
    RegisterReal(ki, "ki", 0.0, ScalarVariable::Type::TunableParameter);
    // fmicpp 0.5 registers Type::TunableParameter as fixed, so set it again here
    variability["kp"] = ScalarVariable::Variability::Tunable;
    variability["ki"] = ScalarVariable::Variability::Tunable;

    RegisterReal(rated_speed_main_engine_rpm,                  "rated_speed_main_engine_rpm",                  0.0);
    RegisterReal(gear_ratio_between_main_engine_and_propeller, "gear_ratio_between_main_engine_and_propeller", 0.0);
    RegisterReal(idle_rpm_fraction,                            "idle_rpm_fraction",                            0.0);

    // Input
    RegisterInput(desired_ship_speed,  "desired_ship_speed",  0.0);
    RegisterInput(measured_ship_speed, "measured_ship_speed", 0.0);

    // Output
    RegisterOutput(shaft_speed_cmd_rpm, "shaft_speed_cmd_rpm", 0.0);
    // - debug
    RegisterOutput(rpm_cmd_pi, "rpm_cmd_pi", 0.0);
    RegisterOutput(rpm_cmd_ff, "rpm_cmd_ff", 0.0);
    RegisterOutput(error_i,    "error_i",    0.0);
    RegisterOutput(error,      "error",      0.0);
    RegisterOutput(des_speed,  "des_speed",  0.0);
    RegisterOutput(mea_speed,  "mea_speed",  0.0);
}

// Saturate the input val such that it remains between "low" and "hi"
static double sat(double val, double low, double hi)
{
    return std::max(low, std::min(val, hi));
}

double MODEL_IDENTIFIER::map_speed_to_rpm(double desired_ship_speed, double shaft_speed_floor_rpm)
{
    // Testing data
    const double rpm_data[9] = { 66, 100, 200, 300, 400, 450, 500, 600, 660};
    const double v_data[9]   = {0.18, 0.407, 1.533, 3.168, 5.14, 6.2, 6.72, 6.72, 6.72};

    // Remove saturated data
    double rpm_lin[9];
    double v_lin[9];
    int n = 0;
    for (int i = 0; i < 9; i++)
    {
        if (v_data[i] < 6.72)
        {
            rpm_lin[n] = rpm_data[i];
            v_lin[n]   = v_data[i];
            n += 1;
        }
    }

    // Get the a by finding which segment desired_ship_speed falls into and return that segment slope
    double vq = std::min(std::max(desired_ship_speed, v_lin[0]), v_lin[n - 1]);  // Get the v_query (np.clip)
    int idx = 0;                                                                   // np.searchsorted(v_lin, vq) - 1
    while (idx < n && v_lin[idx] < vq)
    {
        idx += 1;
    }
    idx = idx - 1;                                                                 // Get the idx in which the v_query fell into (lower bound)
    idx = std::min(std::max(idx, 0), n - 2);                                       // Make sure that the idx do not preceed 0 and exceed the max idx of the data
    double dv = v_lin[idx + 1] - v_lin[idx];                                       // Get the change of ship velocity
    double dr = rpm_lin[idx + 1] - rpm_lin[idx];                                   // Get the change of shaft speed
    double a  = dr / dv;                                                           // Get the rate of rpm change w.r.t. change of ship speed

    // Get the feed forward rpm (from linear mapping)
    // Formula: a [rpm/(m/s)] * desired_ship_speed [m/s] + shaft_speed_floor_rpm [rpm]
    // rpm_cmd_ff = a * desired_ship_speed + shaft_speed_floor_rpm
    double rpm_cmd_ff = a + shaft_speed_floor_rpm;
    return rpm_cmd_ff;
}

void MODEL_IDENTIFIER::do_step(double current_time, double step_size)
{
    try
    {
        // Max shaft speed
        double shaft_speed_max_rpm   = 1.1 * rated_speed_main_engine_rpm * gear_ratio_between_main_engine_and_propeller;
        double shaft_speed_floor_rpm = idle_rpm_fraction * shaft_speed_max_rpm;   // Floor shaft speed ~ 10 % max rpm

        // Get PI rpm command
        double error_new   = desired_ship_speed - measured_ship_speed;
        double rpm_cmd_p   = kp * error_new;
        double rpm_cmd_i   = ki * error_i;
        double rpm_cmd_pi_new = rpm_cmd_p + rpm_cmd_i;

        // Get feedforward rpm command
        double rpm_cmd_ff_new = map_speed_to_rpm(desired_ship_speed, shaft_speed_floor_rpm);

        // Get the unsaturated rpm command
        double rpm_cmd_unsat = rpm_cmd_ff_new + rpm_cmd_pi_new;

        // Saturate the rpm command
        double rpm_cmd_sat = sat(rpm_cmd_unsat, 0.0, shaft_speed_max_rpm);

        // Anti-windup
        bool saturated_high = (rpm_cmd_sat >= shaft_speed_max_rpm - 1e-9);
        bool saturated_low  = (rpm_cmd_sat <= shaft_speed_floor_rpm + 1e-9);

        // If high-saturated and error wants to increase command further -> don't integrate.
        // If low-saturated and error wants to decrease command further -> don't integrate.
        bool pushing_further_high = saturated_high && (error_new > 0.0);
        bool pushing_further_low  = saturated_low  && (error_new < 0.0);

        // If command IS saturated, do not integrate the error
        if (!(pushing_further_high || pushing_further_low))
        {
            error_i += error_new * step_size;
        }

        // - debug
        rpm_cmd_pi = rpm_cmd_pi_new;
        rpm_cmd_ff = rpm_cmd_ff_new;
        error      = error_new;
        des_speed  = desired_ship_speed;
        mea_speed  = measured_ship_speed;

        // Output
        shaft_speed_cmd_rpm = rpm_cmd_sat;
    }
    catch (const std::exception& e)
    {
        // IMPORTANT: do not crash host
        std::cout << "[ShaftSpeedController] ERROR t=" << current_time << " dt=" << step_size << ": " << e.what() << std::endl;

        shaft_speed_cmd_rpm = 0.0;
    }
}

void MODEL_IDENTIFIER::DoStep(const double globalTime)
{
    do_step(time, globalTime - time);
    SystemBaseWithSolvers::DoStep(globalTime);
}
