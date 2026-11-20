#ifndef SPEED_CONTROLLER_H
#define SPEED_CONTROLLER_H
#include "modelName.h"
#include "systemBaseWithSolvers.h"

// Speed Controller C++ FMU implementation.
// Port of FMU_scripts/simplified_ship/speed_controller_fmu.py
class MODEL_IDENTIFIER : public SystemBaseWithSolvers
{
public:
    MODEL_IDENTIFIER(std::shared_ptr<LoggerBase> _logger = {}, std::string _location = "");

    // Runs do_step() once per communication step
    void DoStep(const double globalTime) override;

private:
    void do_step(double current_time, double step_size);
    double pid_ctrl(double setpoint, double measurement, double step_size);

    // PI Parameters
    double kp;
    double ki;
    double kd;

    // Internal Variables
    double error_i;
    double prev_error;

    // Input
    double desired_ship_speed;
    double measured_ship_speed;

    // Output
    double thrust_force;
};

#endif
