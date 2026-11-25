#ifndef MISSION_MANAGER_AST_H
#define MISSION_MANAGER_AST_H
#include "modelName.h"
#include "systemBaseWithSolvers.h"

#include <array>
#include <string>
#include <vector>

// Mission Manager C++ FMU implementation for EB.ASTv2.
// Manages waypoint progression using previous and next setpoints,
// supports zeroeth and sampled intermediate waypoint insertion during time activation,
// and requests external scope-angle inputs to generate new intermediate waypoints
// during runtime. Needs start and end waypoints.
// Port of FMU_scripts/mission_manager/mission_manager_ast_fmu.py
class MODEL_IDENTIFIER : public SystemBaseWithSolvers
{
public:
    MODEL_IDENTIFIER(std::shared_ptr<LoggerBase> _logger = {}, std::string _location = "");

    // Runs do_step() once per communication step
    void DoStep(const double globalTime) override;

private:
    // (north, east, speed) and (north, east), like the tuples in the Python version
    using Waypoint = std::array<double, 3>;
    using Point    = std::array<double, 2>;

    void do_step(double current_time, double step_size);
    void _build_trajectory();
    double _dist2_to_next();
    void _get_route_segment_parameters(Waypoint& base_setpoint, Waypoint& head_setpoint,
                                       double& segment_length, double& beta);
    void _get_inter_wp_parameters(const Waypoint& inter_wp, const Waypoint& base_setpoint,
                                  const Waypoint& head_setpoint, double beta,
                                  double& untraversed_segment_length, Point& inter_wp_proj);
    void _insert_inter_wp_to_traj(const Waypoint& inter_wp);
    void _get_zeroeth_intermediate_waypoint(Waypoint& inter_wp, Point& inter_wp_proj,
                                            double& untraversed_segment_length);
    void _get_intermediate_waypoint(double scope_angle_deg, double scope_length, Waypoint& inter_wp,
                                    Point& inter_wp_proj, double& untraversed_segment_length);
    void _save_intermediate_waypoint(const Waypoint& inter_wp, const Point& inter_wp_proj);
    void _get_prev_and_next_waypoint();

    // Parameters
    double ra;
    double time_to_start_iw_sampler;
    int max_inner_wp;
    int max_sampled_inter_wp;
    double scope_angle_max_deg;

    // Waypoints (parameters)
    double wp_start_north;
    double wp_start_east;
    double wp_start_speed;

    double wp_end_north;
    double wp_end_east;
    double wp_end_speed;

    // Inputs
    double north;
    double east;
    double scope_angle_deg;
    double scope_length;

    // Outputs
    double prev_wp_north;
    double prev_wp_east;
    double prev_wp_speed;
    double next_wp_north;
    double next_wp_east;
    double next_wp_speed;
    bool last_segment_active;
    bool reach_wp_end;
    bool request_captain_intent;
    bool insert_wp_now;
    int idx;
    double next_wp_proj_north;
    double next_wp_proj_east;
    int accepted_sampled_inter_wp;

    // Debug
    std::string messages;

    // Internal
    // Base trajectory
    std::vector<Waypoint> _traj;
    bool _traj_built;

    // For intermediate waypoint sampling
    std::vector<Waypoint> _traj_base;
    int _idx_in_segment;
    int _segment_idx;
    bool _zeroeth_iw_inserted;
    double _temp_untraversed_segment_length;
};

#endif
