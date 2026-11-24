#include "modelName.h"
#include "systemBase.h"
#include SYSTEM_HEADER
#include "guid.h"

#include <cmath>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

MODEL_IDENTIFIER::MODEL_IDENTIFIER(std::shared_ptr<LoggerBase> _logger, std::string _location)
    : SystemBaseWithSolvers(_logger, _location)
{
    author      = "Andreas R.G. Sitorus, Etienne Purcell";
    description = "Ship Model C++ FMU Implementation";
    version     = MODEL_VERSION;
    name        = MODELNAME;
    identifier  = MODEL_IDENTIFIER_STR;
    guid        = fmuGUID;

    // No built-in solver: do_step() does the work
    solverType = "None";
    printInfo  = false;

    // Precomputed Flag (hidden, but saved with the FMU state)
    RegisterInternal(_precomputed, "_precomputed", false);

    // Registration
    // =========================
    // Ship configuration (parameters, fixed)
    // =========================
    RegisterReal(dead_weight_tonnage, "dead_weight_tonnage", 0.0);
    RegisterReal(coefficient_of_deadweight_to_displacement, "coefficient_of_deadweight_to_displacement", 0.0);
    RegisterReal(bunkers, "bunkers", 0.0);
    RegisterReal(ballast, "ballast", 0.0);
    RegisterReal(length_of_ship, "length_of_ship", 0.0);
    RegisterReal(width_of_ship, "width_of_ship", 0.0);
    RegisterReal(added_mass_coefficient_in_surge, "added_mass_coefficient_in_surge", 0.0);
    RegisterReal(added_mass_coefficient_in_sway, "added_mass_coefficient_in_sway", 0.0);
    RegisterReal(added_mass_coefficient_in_yaw, "added_mass_coefficient_in_yaw", 0.0);
    RegisterReal(mass_over_linear_friction_coefficient_in_surge, "mass_over_linear_friction_coefficient_in_surge", 0.0);
    RegisterReal(mass_over_linear_friction_coefficient_in_sway, "mass_over_linear_friction_coefficient_in_sway", 0.0);
    RegisterReal(mass_over_linear_friction_coefficient_in_yaw, "mass_over_linear_friction_coefficient_in_yaw", 0.0);
    RegisterReal(nonlinear_friction_coefficient_in_surge, "nonlinear_friction_coefficient_in_surge", 0.0);
    RegisterReal(nonlinear_friction_coefficient_in_sway, "nonlinear_friction_coefficient_in_sway", 0.0);
    RegisterReal(nonlinear_friction_coefficient_in_yaw, "nonlinear_friction_coefficient_in_yaw", 0.0);

    // =========================
    // Environmental Configuration (parameters, fixed)
    // =========================
    RegisterReal(rho_seawater, "rho_seawater", 1025.0);
    RegisterReal(rho_air, "rho_air", 1.2);
    RegisterReal(front_above_water_height, "front_above_water_height", 0.0);
    RegisterReal(side_above_water_height, "side_above_water_height", 0.0);
    RegisterReal(cx, "cx", 0.0);
    RegisterReal(cy, "cy", 0.0);
    RegisterReal(cn, "cn", 0.0);

    // =========================
    // Initial states (parameters, fixed)
    // =========================
    RegisterReal(initial_north_position_m, "initial_north_position_m", 0.0);
    RegisterReal(initial_east_position_m, "initial_east_position_m", 0.0);
    RegisterReal(initial_yaw_angle_rad, "initial_yaw_angle_rad", 0.0);
    RegisterReal(initial_forward_speed_m_per_s, "initial_forward_speed_m_per_s", 0.0);
    RegisterReal(initial_sideways_speed_m_per_s, "initial_sideways_speed_m_per_s", 0.0);
    RegisterReal(initial_yaw_rate_rad_per_s, "initial_yaw_rate_rad_per_s", 0.0);

    // =========================
    // Input Variables
    // =========================
    RegisterInput(thrust_force, "thrust_force", 0.0);
    RegisterInput(rudder_force_v, "rudder_force_v", 0.0);
    RegisterInput(rudder_force_r, "rudder_force_r", 0.0);
    RegisterInput(wind_speed, "wind_speed", 0.0);
    RegisterInput(wind_dir_rad, "wind_dir_rad", 0.0);
    RegisterInput(current_speed, "current_speed", 0.0);
    RegisterInput(current_dir_rad, "current_dir_rad", 0.0);

    // =========================
    // Output Variables
    // =========================
    RegisterOutput(north, "north", 0.0);
    RegisterOutput(east, "east", 0.0);
    RegisterOutput(yaw_angle_rad, "yaw_angle_rad", 0.0);
    RegisterOutput(forward_speed, "forward_speed", 0.0);
    RegisterOutput(sideways_speed, "sideways_speed", 0.0);
    RegisterOutput(yaw_rate, "yaw_rate", 0.0);

    RegisterOutput(total_ship_speed, "total_ship_speed", 0.0);

    RegisterOutput(d_north, "d_north", 0.0);
    RegisterOutput(d_east, "d_east", 0.0);
    RegisterOutput(d_yaw_angle_rad, "d_yaw_angle_rad", 0.0);
    RegisterOutput(d_forward_speed, "d_forward_speed", 0.0);
    RegisterOutput(d_sideways_speed, "d_sideways_speed", 0.0);
    RegisterOutput(d_yaw_rate, "d_yaw_rate", 0.0);
}

// Safe Division Module
static double safe_div(double num, double den, const std::string& name)
{
    if (std::abs(den) < 1e-9)
    {
        throw std::runtime_error("Division by ~0 in " + name + ": den=" + std::to_string(den));
    }
    return num / den;
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

void MODEL_IDENTIFIER::validate_params()
{
    // Anything that appears in a denominator or matrix must be > 0
    std::vector<std::pair<std::string, double>> must_be_pos = {
        {"coefficient_of_deadweight_to_displacement", coefficient_of_deadweight_to_displacement},
        {"rho_seawater", rho_seawater},
        {"length_of_ship", length_of_ship},
        {"width_of_ship", width_of_ship},
        {"mass_over_linear_friction_coefficient_in_surge", mass_over_linear_friction_coefficient_in_surge},
        {"mass_over_linear_friction_coefficient_in_sway", mass_over_linear_friction_coefficient_in_sway},
        {"mass_over_linear_friction_coefficient_in_yaw", mass_over_linear_friction_coefficient_in_yaw},
    };
    for (const auto& [n, v] : must_be_pos)
    {
        if (v <= 0.0)
        {
            throw std::runtime_error("Parameter '" + n + "' must be > 0, got " + std::to_string(v));
        }
    }

    // optional: sanity checks
    if (dead_weight_tonnage <= 0)
    {
        throw std::runtime_error("dead_weight_tonnage must be > 0, got " + std::to_string(dead_weight_tonnage));
    }
}

void MODEL_IDENTIFIER::pre_compute_initial_variables()
{
    validate_params();

    // Ship
    payload = 0.9 * (dead_weight_tonnage - bunkers);

    // lsw = DWT/c - DWT  (c must be >0)
    lsw = safe_div(dead_weight_tonnage, coefficient_of_deadweight_to_displacement,
                   "lsw: dead_weight_tonnage/coefficient") - dead_weight_tonnage;

    mass   = lsw + payload + bunkers + ballast;
    l_ship = length_of_ship;
    w_ship = width_of_ship;

    // draft t_ship = m/(rho*L*W)
    double denom = rho_seawater * l_ship * w_ship;
    t_ship = safe_div(mass, denom, "t_ship");

    x_g         = 0.0;
    i_z         = mass * (l_ship * l_ship + w_ship * w_ship) / 12.0;
    proj_area_f = w_ship * front_above_water_height;
    proj_area_l = l_ship * side_above_water_height;

    t_surge = mass_over_linear_friction_coefficient_in_surge;
    t_sway  = mass_over_linear_friction_coefficient_in_sway;
    t_yaw   = mass_over_linear_friction_coefficient_in_yaw;
    ku      = nonlinear_friction_coefficient_in_surge;
    kv      = nonlinear_friction_coefficient_in_sway;
    kr      = nonlinear_friction_coefficient_in_yaw;

    Eigen::Vector3d added_mass = set_added_mass(added_mass_coefficient_in_surge,
                                                added_mass_coefficient_in_sway,
                                                added_mass_coefficient_in_yaw);
    x_du = added_mass[0];
    y_dv = added_mass[1];
    n_dr = added_mass[2];
}

// Sets the added mass in surge due to surge motion, sway due to sway motion
// and yaw due to yaw motion according to given coeffs. Returns (x_du, y_dv, n_dr).
Eigen::Vector3d MODEL_IDENTIFIER::set_added_mass(double surge_coeff, double sway_coeff, double yaw_coeff)
{
    double x_du = mass * surge_coeff;
    double y_dv = mass * sway_coeff;
    double n_dr = i_z * yaw_coeff;
    return Eigen::Vector3d(x_du, y_dv, n_dr);
}

// Rotation matrix for rotations about the z-axis, such that
// "body-fixed coordinates" = rotation x "North-east-down-fixed coordinates".
static Eigen::Matrix3d rotation(double yaw_angle_rad)
{
    Eigen::Matrix3d R;
    R << std::cos(yaw_angle_rad), -std::sin(yaw_angle_rad), 0,
         std::sin(yaw_angle_rad),  std::cos(yaw_angle_rad), 0,
         0, 0, 1;
    return R;
}

Eigen::Matrix3d MODEL_IDENTIFIER::mass_matrix()
{
    Eigen::Matrix3d M;
    M << mass + x_du, 0, 0,
         0, mass + y_dv, mass * x_g,
         0, mass * x_g, i_z + n_dr;
    return M;
}

Eigen::Matrix3d MODEL_IDENTIFIER::coriolis_matrix(double forward_speed, double sideways_speed, double yaw_rate)
{
    Eigen::Matrix3d C;
    C << 0, 0, -mass * (x_g * yaw_rate + sideways_speed),
         0, 0, mass * forward_speed,
         mass * (x_g * yaw_rate + sideways_speed), -mass * forward_speed, 0;
    return C;
}

Eigen::Matrix3d MODEL_IDENTIFIER::coriolis_added_mass_matrix(double u_r, double v_r)
{
    Eigen::Matrix3d C;
    C << 0, 0, y_dv * v_r,
         0, 0, -x_du * u_r,
         -y_dv * v_r, x_du * u_r, 0;
    return C;
}

Eigen::Matrix3d MODEL_IDENTIFIER::linear_damping_matrix()
{
    Eigen::Matrix3d D;
    D << mass / t_surge, 0, 0,
         0, mass / t_sway, 0,
         0, 0, i_z / t_yaw;
    return D;
}

Eigen::Matrix3d MODEL_IDENTIFIER::non_linear_damping_matrix(double forward_speed, double sideways_speed, double yaw_rate)
{
    Eigen::Matrix3d D;
    D << ku * forward_speed, 0, 0,
         0, kv * sideways_speed, 0,
         0, 0, kr * yaw_rate;
    return D;
}

// Calculates the forces due to the relative wind speed, acting on the ship
// in surge, sway and yaw direction.
Eigen::Vector3d MODEL_IDENTIFIER::get_wind_force(double wind_speed, double wind_dir_rad, double yaw_angle_rad,
                                                 double forward_speed, double sideways_speed)
{
    double uw        = wind_speed * std::sin(wind_dir_rad - yaw_angle_rad);
    double vw        = wind_speed * std::cos(wind_dir_rad - yaw_angle_rad);
    double u_rw      = uw - forward_speed;
    double v_rw      = vw - sideways_speed;
    double gamma_rw  = -std::atan2(v_rw, u_rw);
    double wind_rw2  = u_rw * u_rw + v_rw * v_rw;
    double c_x       = -cx * std::sin(gamma_rw);
    double c_y       = cy * std::cos(gamma_rw);
    double c_n       = cn * std::sin(2 * gamma_rw);
    double tau_coeff = 0.5 * rho_air * wind_rw2;
    double tau_u     = tau_coeff * c_x * proj_area_f;
    double tau_v     = tau_coeff * c_y * proj_area_l;
    double tau_n     = tau_coeff * c_n * proj_area_l * l_ship;
    return Eigen::Vector3d(tau_u, tau_v, tau_n);
}

// Time derivatives of the north position, east position and yaw angle.
// Returns (d_north, d_east, d_yaw_angle_rad).
Eigen::Vector3d MODEL_IDENTIFIER::three_dof_kinematics(double yaw_angle_rad, double forward_speed,
                                                       double sideways_speed, double yaw_rate)
{
    Eigen::Vector3d vel(forward_speed, sideways_speed, yaw_rate);
    return rotation(yaw_angle_rad) * vel;
}

// Accelerations of the ship, as a function of thrust-force, rudder forces, wind forces
// and the states in the previous time-step. Returns (d_forward_speed, d_sideways_speed, d_yaw_rate).
Eigen::Vector3d MODEL_IDENTIFIER::three_dof_kinetics(double yaw_angle_rad, double forward_speed, double sideways_speed,
                                                     double yaw_rate, double thrust_force, double rudder_force_v,
                                                     double rudder_force_r, const Eigen::Vector3d& wind_force,
                                                     const Eigen::Vector3d& current_velocity)
{
    // Forces acting
    Eigen::Vector3d ctrl_force(thrust_force, rudder_force_v, rudder_force_r);

    // assembling state vector
    Eigen::Vector3d vel(forward_speed, sideways_speed, yaw_rate);

    // Transforming current velocity to ship frame
    Eigen::Vector3d v_c = rotation(yaw_angle_rad).inverse() * current_velocity;
    double u_r = forward_speed - v_c[0];
    double v_r = sideways_speed - v_c[1];

    // Kinetic equation
    Eigen::Matrix3d M   = mass_matrix();
    Eigen::Vector3d rhs =
        - coriolis_matrix(forward_speed, sideways_speed, yaw_rate) * vel
        - coriolis_added_mass_matrix(u_r, v_r) * (vel - v_c)
        - (linear_damping_matrix() + non_linear_damping_matrix(forward_speed, sideways_speed, yaw_rate)) * (vel - v_c)
        + wind_force + ctrl_force;
    // LU with partial pivoting, like np.linalg.solve
    return M.partialPivLu().solve(rhs);
}

void MODEL_IDENTIFIER::do_step(double current_time, double step_size)
{
    try
    {
        // Precompute once
        if (!_precomputed)
        {
            // Pre-compute initial variables
            pre_compute_initial_variables();

            // Initialize the parameters
            north          = initial_north_position_m;
            east           = initial_east_position_m;
            yaw_angle_rad  = initial_yaw_angle_rad;
            forward_speed  = initial_forward_speed_m_per_s;
            sideways_speed = initial_sideways_speed_m_per_s;
            yaw_rate       = initial_yaw_rate_rad_per_s;

            // Set the flag as true
            _precomputed = true;
        }

        // Get current ship states
        double north_0          = north;
        double east_0           = east;
        double yaw_angle_rad_0  = yaw_angle_rad;
        double forward_speed_0  = forward_speed;
        double sideways_speed_0 = sideways_speed;
        double yaw_rate_0       = yaw_rate;

        // Get force
        Eigen::Vector3d wind_force = get_wind_force(wind_speed, wind_dir_rad, yaw_angle_rad_0,
                                                    forward_speed_0, sideways_speed_0);
        Eigen::Vector3d current_velocity(current_speed * std::cos(current_dir_rad),
                                         current_speed * std::sin(current_dir_rad),
                                         0.0);

        // Compute the kinematics and kinetics
        Eigen::Vector3d d_pos = three_dof_kinematics(yaw_angle_rad_0, forward_speed_0, sideways_speed_0, yaw_rate_0);
        Eigen::Vector3d d_vel = three_dof_kinetics(yaw_angle_rad_0, forward_speed_0, sideways_speed_0, yaw_rate_0,
                                                   thrust_force, rudder_force_v, rudder_force_r,
                                                   wind_force, current_velocity);

        // Get Output
        // Compute d_x
        d_north          = d_pos[0];
        d_east           = d_pos[1];
        d_yaw_angle_rad  = d_pos[2];
        d_forward_speed  = d_vel[0];
        d_sideways_speed = d_vel[1];
        d_yaw_rate       = d_vel[2];

        // Integrate d_x (Euler integration)
        north          = north_0          + step_size * d_north;
        east           = east_0           + step_size * d_east;
        yaw_angle_rad  = wrap_to_pi(yaw_angle_rad_0 + step_size * d_yaw_angle_rad);
        forward_speed  = forward_speed_0  + step_size * d_forward_speed;
        sideways_speed = sideways_speed_0 + step_size * d_sideways_speed;
        yaw_rate       = yaw_rate_0       + step_size * d_yaw_rate;

        // Get the measured speed
        total_ship_speed = std::sqrt(forward_speed * forward_speed + sideways_speed * sideways_speed);
    }
    catch (const std::exception& e)
    {
        // IMPORTANT: do not crash host
        std::cout << "[ShipModel] ERROR t=" << current_time << " dt=" << step_size << ": " << e.what() << std::endl;

        // Freeze dynamics safely (keep last state/outputs)
        d_north = d_east = d_yaw_angle_rad = 0.0;
        d_forward_speed = d_sideways_speed = d_yaw_rate = 0.0;
        // keep measured speed consistent with current outputs
        total_ship_speed = std::isfinite(forward_speed) ? std::hypot(forward_speed, sideways_speed) : 0.0;
    }
}

void MODEL_IDENTIFIER::DoStep(const double globalTime)
{
    do_step(time, globalTime - time);
    SystemBaseWithSolvers::DoStep(globalTime);
}
