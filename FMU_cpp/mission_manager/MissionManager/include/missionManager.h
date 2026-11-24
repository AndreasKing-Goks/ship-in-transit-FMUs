#ifndef MISSION_MANAGER_H
#define MISSION_MANAGER_H
#include "modelName.h"
#include "systemBaseWithSolvers.h"

// Mission Manager C++ FMU implementation.
// Manages a list of mission (waypoints and desired speeds) and provides the previous and next setpoints.
// It includes a switching mechanism based on a radius of acceptance.
// Minimum 2 setpoints, maximum 10 setpoints.
// Port of FMU_scripts/mission_manager/mission_manager_fmu.py
class MODEL_IDENTIFIER : public SystemBaseWithSolvers
{
public:
    MODEL_IDENTIFIER(std::shared_ptr<LoggerBase> _logger = {}, std::string _location = "");

    // Runs do_step() once per communication step
    void DoStep(const double globalTime) override;

private:
    void do_step(double current_time, double step_size);
    void _build_trajectory();
    void _add_to_traj(double n, double e, double s);
    double _dist2_to_next();

    // Parameters
    double ra;
    int max_inner_wp;

    // Waypoints (parameters)
    double wp_start_north;
    double wp_start_east;
    double wp_start_speed;

    double wp_north[8];  // wp_1_north .. wp_8_north
    double wp_east[8];   // wp_1_east  .. wp_8_east
    double wp_speed[8];  // wp_1_speed .. wp_8_speed

    double wp_end_north;
    double wp_end_east;
    double wp_end_speed;

    // Inputs
    double north;
    double east;

    // Outputs
    double prev_wp_north;
    double prev_wp_east;
    double prev_wp_speed;

    double next_wp_north;
    double next_wp_east;
    double next_wp_speed;

    bool last_wp_active;

    bool reach_wp_end;

    // Internal
    // The trajectory: start, up to 8 inner waypoints, end
    double _traj_north[10] = {};
    double _traj_east[10]  = {};
    double _traj_speed[10] = {};
    int _traj_len = 0;
    int _idx;
    bool _traj_built;
};

#endif
