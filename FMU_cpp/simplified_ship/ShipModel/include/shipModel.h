#ifndef SHIP_MODEL_H
#define SHIP_MODEL_H
#include "modelName.h"
#include "systemBaseWithSolvers.h"

#include <Eigen/Dense>

// Ship Model C++ FMU implementation.
// Kinetic and kinematic models of the ship.
// Port of FMU_scripts/simplified_ship/ship_model_fmu.py
class MODEL_IDENTIFIER : public SystemBaseWithSolvers
{
public:
    MODEL_IDENTIFIER(std::shared_ptr<LoggerBase> _logger = {}, std::string _location = "");

    // Runs do_step() once per communication step
    void DoStep(const double globalTime) override;

private:
    void do_step(double current_time, double step_size);
    void validate_params();
    void pre_compute_initial_variables();
    Eigen::Vector3d set_added_mass(double surge_coeff, double sway_coeff, double yaw_coeff);
    Eigen::Matrix3d mass_matrix();
    Eigen::Matrix3d coriolis_matrix(double forward_speed, double sideways_speed, double yaw_rate);
    Eigen::Matrix3d coriolis_added_mass_matrix(double u_r, double v_r);
    Eigen::Matrix3d linear_damping_matrix();
    Eigen::Matrix3d non_linear_damping_matrix(double forward_speed, double sideways_speed, double yaw_rate);
    Eigen::Vector3d get_wind_force(double wind_speed, double wind_dir_rad, double yaw_angle_rad,
                                   double forward_speed, double sideways_speed);
    Eigen::Vector3d three_dof_kinematics(double yaw_angle_rad, double forward_speed, double sideways_speed,
                                         double yaw_rate);
    Eigen::Vector3d three_dof_kinetics(double yaw_angle_rad, double forward_speed, double sideways_speed,
                                       double yaw_rate, double thrust_force, double rudder_force_v,
                                       double rudder_force_r, const Eigen::Vector3d& wind_force,
                                       const Eigen::Vector3d& current_velocity);

    // Precomputed Flag
    bool _precomputed;

    // Parameters
    // Ship configuration
    double dead_weight_tonnage;
    double coefficient_of_deadweight_to_displacement;
    double bunkers;
    double ballast;
    double length_of_ship;
    double width_of_ship;
    double added_mass_coefficient_in_surge;
    double added_mass_coefficient_in_sway;
    double added_mass_coefficient_in_yaw;
    double mass_over_linear_friction_coefficient_in_surge;
    double mass_over_linear_friction_coefficient_in_sway;
    double mass_over_linear_friction_coefficient_in_yaw;
    double nonlinear_friction_coefficient_in_surge;
    double nonlinear_friction_coefficient_in_sway;
    double nonlinear_friction_coefficient_in_yaw;

    // Environment
    double rho_seawater;
    double rho_air;
    double g = 9.81;
    double front_above_water_height;
    double side_above_water_height;
    double cx;
    double cy;
    double cn;

    // Initial states
    double initial_north_position_m;
    double initial_east_position_m;
    double initial_yaw_angle_rad;
    double initial_forward_speed_m_per_s;
    double initial_sideways_speed_m_per_s;
    double initial_yaw_rate_rad_per_s;

    // Input variables
    double thrust_force;
    double rudder_force_v;
    double rudder_force_r;
    double wind_speed;
    double wind_dir_rad;
    double current_speed;
    double current_dir_rad;

    // Output variables
    double north;
    double east;
    double yaw_angle_rad;
    double forward_speed;
    double sideways_speed;
    double yaw_rate;

    double total_ship_speed;

    double d_north;
    double d_east;
    double d_yaw_angle_rad;
    double d_forward_speed;
    double d_sideways_speed;
    double d_yaw_rate;

    // Computed once by pre_compute_initial_variables()
    double payload = 0.0;
    double lsw = 0.0;
    double mass = 0.0;
    double l_ship = 0.0;
    double w_ship = 0.0;
    double t_ship = 0.0;
    double x_g = 0.0;
    double i_z = 0.0;
    double proj_area_f = 0.0;
    double proj_area_l = 0.0;
    double t_surge = 0.0;
    double t_sway = 0.0;
    double t_yaw = 0.0;
    double ku = 0.0;
    double kv = 0.0;
    double kr = 0.0;
    double x_du = 0.0;
    double y_dv = 0.0;
    double n_dr = 0.0;
};

#endif
