#include "modelName.h"
#include "systemBase.h"
#include SYSTEM_HEADER
#include "guid.h"

#include <iostream>

MODEL_IDENTIFIER::MODEL_IDENTIFIER(std::shared_ptr<LoggerBase> _logger, std::string _location)
    : SystemBaseWithSolvers(_logger, _location)
{
    author      = "Andreas R.G. Sitorus, Etienne Purcell";
    description = "PI-Based Speed Controller C++ FMU Implementation";
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
    RegisterReal(kd, "kd", 0.0, ScalarVariable::Type::TunableParameter);
    // fmicpp 0.5 registers Type::TunableParameter as fixed, so set it again here
    variability["kp"] = ScalarVariable::Variability::Tunable;
    variability["ki"] = ScalarVariable::Variability::Tunable;
    variability["kd"] = ScalarVariable::Variability::Tunable;

    // Internal Variables (hidden, but saved with the FMU state)
    RegisterInternal(error_i,    "error_i",    0.0);
    RegisterInternal(prev_error, "prev_error", 0.0);

    // Input
    RegisterInput(desired_ship_speed,  "desired_ship_speed",  0.0);
    RegisterInput(measured_ship_speed, "measured_ship_speed", 0.0);

    // Output
    RegisterOutput(thrust_force, "thrust_force", 0.0);
}

// Uses a proportional-derivative-integral control law to calculate a control output.
// The Python version also has optional integrator limits, but they are never used.
double MODEL_IDENTIFIER::pid_ctrl(double setpoint, double measurement, double step_size)
{
    double error   = setpoint - measurement;
    double d_error = step_size > 0 ? (error - prev_error) / step_size : 0.0;
    double error_i_new = error_i + error * step_size;

    prev_error = error;
    error_i    = error_i_new;
    return (error * kp + d_error * kd + error_i_new * ki);
}

void MODEL_IDENTIFIER::do_step(double current_time, double step_size)
{
    try
    {
        // Get thrust force
        thrust_force = pid_ctrl(desired_ship_speed, measured_ship_speed, step_size);
    }
    catch (const std::exception& e)
    {
        // IMPORTANT: do not crash host
        std::cout << "[SpeedController] ERROR t=" << current_time << " dt=" << step_size << ": " << e.what() << std::endl;

        thrust_force = 0.0;
    }
}

void MODEL_IDENTIFIER::DoStep(const double globalTime)
{
    do_step(time, globalTime - time);
    SystemBaseWithSolvers::DoStep(globalTime);
}
