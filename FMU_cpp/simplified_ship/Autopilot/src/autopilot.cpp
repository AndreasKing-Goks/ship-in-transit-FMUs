#include "modelName.h"
#include "systemBase.h"
#include SYSTEM_HEADER
#include "guid.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numbers>

MODEL_IDENTIFIER::MODEL_IDENTIFIER(std::shared_ptr<LoggerBase> _logger, std::string _location)
    : SystemBaseWithSolvers(_logger, _location)
{
    author      = "Andreas R.G. Sitorus, Etienne Purcell";
    description = "PID-Based Heading Controller with Line-of-Sight Guidance Algorithm";
    version     = MODEL_VERSION;
    name        = MODELNAME;
    identifier  = MODEL_IDENTIFIER_STR;
    guid        = fmuGUID;

    // No built-in solver: do_step() does the work
    solverType = "None";
    printInfo  = false;

    // Registration
    // LOS Guidance Parameters
    RegisterReal(r,                "r",                0.0, ScalarVariable::Type::TunableParameter);
    RegisterReal(ki_ct,            "ki_ct",            0.0);
    RegisterReal(integrator_limit, "integrator_limit", 0.0);

    // PID Parameters
    RegisterReal(kp, "kp", 0.0, ScalarVariable::Type::TunableParameter);
    RegisterReal(ki, "ki", 0.0, ScalarVariable::Type::TunableParameter);
    RegisterReal(kd, "kd", 0.0, ScalarVariable::Type::TunableParameter);
    RegisterReal(max_rudder_rate_deg_per_sec, "max_rudder_rate_deg_per_sec", 0.0);
    RegisterReal(max_rudder_angle_deg,        "max_rudder_angle_deg",        0.0);
    // fmicpp 0.5 registers Type::TunableParameter as fixed, so set it again here
    variability["r"]  = ScalarVariable::Variability::Tunable;
    variability["kp"] = ScalarVariable::Variability::Tunable;
    variability["ki"] = ScalarVariable::Variability::Tunable;
    variability["kd"] = ScalarVariable::Variability::Tunable;

    // Internal Variables (hidden, but saved with the FMU state)
    RegisterInternal(e_ct_int,              "e_ct_int",              0.0);
    RegisterInternal(error_i,               "error_i",               0.0);
    RegisterInternal(prev_error,            "prev_error",            0.0);
    RegisterInternal(prev_rudder_angle_rad, "prev_rudder_angle_rad", 0.0);

    // Input
    RegisterInput(north,         "north",         0.0);
    RegisterInput(east,          "east",          0.0);
    RegisterInput(yaw_angle_rad, "yaw_angle_rad", 0.0);

    RegisterInput(next_wp_north, "next_wp_north", 0.0);
    RegisterInput(next_wp_east,  "next_wp_east",  0.0);

    RegisterInput(prev_wp_north, "prev_wp_north", 0.0);
    RegisterInput(prev_wp_east,  "prev_wp_east",  0.0);

    // Output
    RegisterOutput(yaw_angle_ref_rad, "yaw_angle_ref_rad", 0.0);
    RegisterOutput(rudder_angle_deg,  "rudder_angle_deg",  0.0);
    RegisterOutput(e_ct,              "e_ct",              0.0);
}

// Same as Python: (a + pi) % (2*pi) - pi.
// Python's % keeps the sign of 2*pi, so a negative result from fmod is moved up by 2*pi.
static double wrap_to_pi(double a)
{
    const double pi = std::numbers::pi;
    double m = std::fmod(a + pi, 2 * pi);
    if (m < 0.0)
    {
        m += 2 * pi;
    }
    return m - pi;
}

// Saturate the input val such that it remains between "low" and "hi"
static double sat(double val, double low, double hi)
{
    return std::max(low, std::min(val, hi));
}

// Same as Python math.radians and math.degrees
static double radians(double deg) { return deg * (std::numbers::pi / 180.0); }
static double degrees(double rad) { return rad * (180.0 / std::numbers::pi); }

// Uses a proportional-derivative-integral control law to calculate a control output.
// The Python version also has optional integrator limits, but they are never used.
double MODEL_IDENTIFIER::pid_ctrl(double setpoint, double measurement, double step_size)
{
    double error   = wrap_to_pi(setpoint - measurement);  // Compute angle error, wrap to pi
    double d_error = step_size > 0 ? (error - prev_error) / step_size : 0.0;
    double error_i_new = error_i + error * step_size;

    prev_error = error;
    error_i    = error_i_new;
    return -(error * kp + d_error * kd + error_i_new * ki);
}

// Limit |du| <= max_rudder_rate * dt, then hard-limit angle.
double MODEL_IDENTIFIER::apply_slew_limit(double u_des_rad, double u_prev_rad, double step_size)
{
    double max_rate_rad  = radians(max_rudder_rate_deg_per_sec);
    double max_angle_rad = radians(max_rudder_angle_deg);

    // Compute the maximum rudder angle displacement based on the maximum rudder angle rate
    double max_delta = max_rate_rad * step_size;

    // Saturate the command input (desired - previous rudder angle command) w.r.t. the max_delta
    double delta = sat(u_des_rad - u_prev_rad, -max_delta, +max_delta);

    // Add the delta to the previous rudder angle command
    double u_cmd = u_prev_rad + delta;

    // Final angle saturation
    return sat(u_cmd, -max_angle_rad, +max_angle_rad);
}

void MODEL_IDENTIFIER::los_guidance(double north, double east, double next_wp_north, double next_wp_east,
                                    double prev_wp_north, double prev_wp_east,
                                    double& yaw_angle_ref_rad, double& e_ct)
{
    double dx = next_wp_east - prev_wp_east;
    double dy = next_wp_north - prev_wp_north;

    double alpha_k = std::atan2(dx, dy);

    // Cross-Track Error
    double v_norm = std::hypot(dx, dy);
    e_ct = -(dx * (north - prev_wp_north) - dy * (east - prev_wp_east)) / std::max(1e-9, v_norm);

    // Keep inside circle
    if (e_ct * e_ct >= r * r && r > 0)
    {
        e_ct = std::copysign(0.99 * r, e_ct);
    }

    double inside = r * r - e_ct * e_ct;
    inside = std::max(0.0, inside);
    double delta = std::max(1e-6, std::sqrt(inside));

    // Integrator anti-windup
    double candidate = e_ct_int + e_ct / delta;
    if (std::abs(candidate) <= integrator_limit)
    {
        e_ct_int = candidate;
    }

    double chi_r = std::atan2(-e_ct, delta - e_ct_int * ki_ct);

    yaw_angle_ref_rad = wrap_to_pi(alpha_k + chi_r);
}

void MODEL_IDENTIFIER::do_step(double current_time, double step_size)
{
    try
    {
        // Get the heading reference and the measured heading using input
        double yaw_angle_ref_rad_new, e_ct_new;
        los_guidance(north, east, next_wp_north, next_wp_east, prev_wp_north, prev_wp_east,
                     yaw_angle_ref_rad_new, e_ct_new);

        // Compute the control signal
        double rudder_angle_des_rad = pid_ctrl(yaw_angle_ref_rad_new, yaw_angle_rad, step_size);

        // Apply slew limiter around the previous rudder command
        double rudder_angle_rad = apply_slew_limit(rudder_angle_des_rad, prev_rudder_angle_rad, step_size);

        // Store current to previous rudder angle command
        prev_rudder_angle_rad = rudder_angle_rad;

        // Outputs
        yaw_angle_ref_rad = yaw_angle_ref_rad_new;
        e_ct              = e_ct_new;
        rudder_angle_deg  = degrees(rudder_angle_rad);
    }
    catch (const std::exception& e)
    {
        // IMPORTANT: do not crash host
        std::cout << "[Autopilot] ERROR t=" << current_time << " dt=" << step_size << ": " << e.what() << std::endl;

        yaw_angle_ref_rad = 0.0;
        e_ct              = 0.0;
        rudder_angle_deg  = 0.0;
    }
}

void MODEL_IDENTIFIER::DoStep(const double globalTime)
{
    do_step(time, globalTime - time);
    SystemBaseWithSolvers::DoStep(globalTime);
}
