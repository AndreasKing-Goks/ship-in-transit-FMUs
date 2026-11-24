#ifndef SHAFT_SPEED_CONTROLLER_H
#define SHAFT_SPEED_CONTROLLER_H
#include "modelName.h"
#include "systemBaseWithSolvers.h"

// Shaft Speed Controller C++ FMU implementation.
// Shaft speed controller based on PI Controller.
// Port of FMU_scripts/ship/shaft_speed_controller_fmu.py
class MODEL_IDENTIFIER : public SystemBaseWithSolvers
{
public:
    MODEL_IDENTIFIER(std::shared_ptr<LoggerBase> _logger = {}, std::string _location = "");

    // Runs do_step() once per communication step
    void DoStep(const double globalTime) override;

private:
    void do_step(double current_time, double step_size);
    double map_speed_to_rpm(double desired_ship_speed, double shaft_speed_floor_rpm);

    // PI Parameters
    double kp;
    double ki;

    double rated_speed_main_engine_rpm;
    double gear_ratio_between_main_engine_and_propeller;
    double idle_rpm_fraction;

    // Internal Variables (error_i is also a debug output)
    double error_i;

    // Input
    double desired_ship_speed;
    double measured_ship_speed;

    // Output
    double shaft_speed_cmd_rpm;

    double rpm_cmd_pi;
    double rpm_cmd_ff;
    double error;

    double des_speed;
    double mea_speed;
};

#endif
