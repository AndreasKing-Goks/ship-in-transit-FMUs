#ifndef SIMPLE_COLLISION_AVOIDANCE_H
#define SIMPLE_COLLISION_AVOIDANCE_H
#include "modelName.h"
#include "systemBaseWithSolvers.h"

// Simple Collision Avoidance C++ FMU implementation.
// Simple collision avoidance algorithm for up to three target ships.
// Port of FMU_scripts/ship/simple_collision_avoidance_fmu.py
class MODEL_IDENTIFIER : public SystemBaseWithSolvers
{
public:
    MODEL_IDENTIFIER(std::shared_ptr<LoggerBase> _logger = {}, std::string _location = "");

    // Runs do_step() once per communication step
    void DoStep(const double globalTime) override;

private:
    void do_step(double current_time, double step_size);

    // Parameters
    double throttle_scale_factor;
    double rud_ang_increment_deg;
    double danger_zone_radius;     // 0.5 nautical mile
    double collision_zone_radius;  // Ideally ship length
    int max_target_ship_count;     // Up to three
    double hold_time;              // Max time for holding one target ship
    double T_lookahead;            // Soon to collide (second)

    // Input
    double own_north;
    double own_east;
    double own_yaw_angle;
    double own_measured_speed;

    double tar_north[3];           // tar_1_north .. tar_3_north
    double tar_east[3];            // tar_1_east  .. tar_3_east
    double tar_yaw_angle[3];       // tar_1_yaw_angle .. tar_3_yaw_angle
    double tar_measured_speed[3];  // tar_1_measured_speed .. tar_3_measured_speed

    double throttle_cmd;
    double rudder_angle_deg;

    // Output
    double new_throttle_cmd;
    double new_rudder_angle_deg;
    double colav_rud_ang_increment;
    bool colav_active;
    bool ship_collision;
    int ship_priority_idx;

    double beta_own_to_tar[3];     // beta_own_to_tar_1 .. beta_own_to_tar_3
    double tcpa_own_to_tar[3];
    double dcpa_own_to_tar[3];
    double dist_own_to_tar[3];
    double rr_own_to_tar[3];

    // Internal Variable
    bool prioritize_one_target;
    int held_target_idx;
    double timer;
    double colav_rudder_sign;
};

#endif
