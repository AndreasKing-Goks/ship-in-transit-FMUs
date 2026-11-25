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
    description = "PI-Based Throttle Controller C++ FMU Implementation";
    version     = MODEL_VERSION;
    name        = MODELNAME;
    identifier  = MODEL_IDENTIFIER_STR;
    guid        = fmuGUID;

    // No built-in solver: do_step() does the work
    solverType = "None";
    printInfo  = false;

    // Registration
    // PI Parameters
    RegisterReal(kp, "kp", 0.0);
    RegisterReal(ki, "ki", 0.0);

    // Internal Variables (hidden, but saved with the FMU state)
    RegisterInternal(error_i, "error_i", 0.0);

    // Input
    RegisterInput(desired_shaft_speed_rpm,  "desired_shaft_speed_rpm",  0.0);
    RegisterInput(measured_shaft_speed_rpm, "measured_shaft_speed_rpm", 0.0);

    // Output
    RegisterOutput(throttle_cmd, "throttle_cmd", 0.0);
}

// Saturate the input val such that it remains between "low" and "hi"
static double sat(double val, double low, double hi)
{
    return std::max(low, std::min(val, hi));
}

void MODEL_IDENTIFIER::do_step(double current_time, double step_size)
{
    try
    {
        // error in RPM domain (inner loop)
        double error = desired_shaft_speed_rpm - measured_shaft_speed_rpm;

        // PI unsaturated
        double u_unsat = kp * error + ki * error_i;

        // Saturate throttle (recommended)
        double u_sat = sat(u_unsat, 0.0, 1.0);

        // Anti-windup: conditional integration
        bool sat_high = (u_sat >= 1.0 - 1e-9);
        bool sat_low  = (u_sat <= 0.0 + 1e-9);

        bool pushing_further_high = sat_high && (error > 0.0);  // wants more throttle but already max
        bool pushing_further_low  = sat_low  && (error < 0.0);  // wants less throttle but already min

        if (!(pushing_further_high || pushing_further_low))
        {
            error_i += error * step_size;
        }

        throttle_cmd = u_sat;
    }
    catch (const std::exception& e)
    {
        // IMPORTANT: do not crash host
        std::cout << "[ThrottleController] ERROR t=" << current_time << " dt=" << step_size << ": " << e.what() << std::endl;

        throttle_cmd = 0.0;
    }
}

void MODEL_IDENTIFIER::DoStep(const double globalTime)
{
    do_step(time, globalTime - time);
    SystemBaseWithSolvers::DoStep(globalTime);
}
