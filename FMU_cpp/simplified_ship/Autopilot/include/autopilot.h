#ifndef AUTOPILOT_H
#define AUTOPILOT_H
#include "modelName.h"
#include "systemBaseWithSolvers.h"

// Autopilot C++ FMU implementation.
// Heading controller (autopilot) based on PID Controller with Line-of-Sight Guidance.
// Port of FMU_scripts/simplified_ship/autopilot_fmu.py
class MODEL_IDENTIFIER : public SystemBaseWithSolvers
{
public:
    MODEL_IDENTIFIER(std::shared_ptr<LoggerBase> _logger = {}, std::string _location = "");

    // Runs do_step() once per communication step
    void DoStep(const double globalTime) override;

private:
    void do_step(double current_time, double step_size);
    double pid_ctrl(double setpoint, double measurement, double step_size);
    double apply_slew_limit(double u_des_rad, double u_prev_rad, double step_size);
    void los_guidance(double north, double east, double next_wp_north, double next_wp_east,
                      double prev_wp_north, double prev_wp_east, double& yaw_angle_ref_rad, double& e_ct);

    // LOS Guidance Parameters
    double r;                  // Lookahead Distance
    double ki_ct;              // Integral Gain
    double integrator_limit;   // Integrator Windup Limit

    // Autopilot Parameters
    double kp;
    double ki;
    double kd;

    double max_rudder_rate_deg_per_sec;
    double max_rudder_angle_deg;

    // Internal Variables
    double e_ct_int;               // Cross Track Error Integral Value
    double error_i;
    double prev_error;
    double prev_rudder_angle_rad;  // Previous Rudder Angle Command, initiated at zero degree

    // Input
    double north;
    double east;
    double yaw_angle_rad;

    double next_wp_north;
    double next_wp_east;

    double prev_wp_north;
    double prev_wp_east;

    // Output
    double yaw_angle_ref_rad;  // Heading Reference
    double rudder_angle_deg;   // Rudder Angle
    double e_ct;               // Cross-Track Error
};

#endif
