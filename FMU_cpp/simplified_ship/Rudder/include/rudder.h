#ifndef RUDDER_H
#define RUDDER_H
#include "modelName.h"
#include "systemBaseWithSolvers.h"

// Ship's Rudder C++ FMU implementation.
// Port of FMU_scripts/simplified_ship/rudder_model_fmu.py
class MODEL_IDENTIFIER : public SystemBaseWithSolvers
{
public:
    MODEL_IDENTIFIER(std::shared_ptr<LoggerBase> _logger = {}, std::string _location = "");

    // Runs do_step() once per communication step
    void DoStep(const double globalTime) override;

private:
    void do_step(double current_time, double step_size);
    void get_rudder_force(double rudder_angle_deg, double yaw_angle_rad, double forward_speed,
                          const double vel_current_ned[3], double& v_force, double& r_force);

    // Parameters (DEGREES for limits)
    double rudder_angle_to_sway_force_coefficient;
    double rudder_angle_to_yaw_force_coefficient;
    double max_rudder_angle_negative_deg;
    double max_rudder_angle_positive_deg;

    // Inputs
    double rudder_angle_deg;
    double yaw_angle_rad;
    double forward_speed;
    double current_speed;
    double current_dir_rad;

    // Outputs
    double rudder_force_v;
    double rudder_force_r;
};

#endif
