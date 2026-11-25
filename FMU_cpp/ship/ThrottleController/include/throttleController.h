#ifndef THROTTLE_CONTROLLER_H
#define THROTTLE_CONTROLLER_H
#include "modelName.h"
#include "systemBaseWithSolvers.h"

// Throttle Controller C++ FMU implementation.
// Throttle controller based on PI Controller.
// Port of FMU_scripts/ship/throttle_controller_fmu.py
class MODEL_IDENTIFIER : public SystemBaseWithSolvers
{
public:
    MODEL_IDENTIFIER(std::shared_ptr<LoggerBase> _logger = {}, std::string _location = "");

    // Runs do_step() once per communication step
    void DoStep(const double globalTime) override;

private:
    void do_step(double current_time, double step_size);

    // PI Parameters
    double kp;
    double ki;

    // Internal Variables
    double error_i;

    // Input
    double desired_shaft_speed_rpm;
    double measured_shaft_speed_rpm;

    // Output
    double throttle_cmd;
};

#endif
