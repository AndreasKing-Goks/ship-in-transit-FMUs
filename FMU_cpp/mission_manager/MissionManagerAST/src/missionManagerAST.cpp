#include "modelName.h"
#include "systemBase.h"
#include SYSTEM_HEADER
#include "guid.h"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <string>

MODEL_IDENTIFIER::MODEL_IDENTIFIER(std::shared_ptr<LoggerBase> _logger, std::string _location)
    : SystemBaseWithSolvers(_logger, _location)
{
    author      = "Andreas R.G. Sitorus, Etienne Purcell";
    description = "Mission Manager for EB-ASTv2";
    version     = MODEL_VERSION;
    name        = MODELNAME;
    identifier  = MODEL_IDENTIFIER_STR;
    guid        = fmuGUID;

    // No built-in solver: do_step() does the work
    solverType = "None";
    printInfo  = false;

    // Internal (hidden, but saved with the FMU state)
    RegisterInternal(_traj_built,                      "_traj_built",                      false);
    RegisterInternal(_idx_in_segment,                  "_idx_in_segment",                  1);
    RegisterInternal(_segment_idx,                     "_segment_idx",                     1);
    RegisterInternal(_zeroeth_iw_inserted,             "_zeroeth_iw_inserted",             false);
    RegisterInternal(_temp_untraversed_segment_length, "_temp_untraversed_segment_length", 0.0);

    // Registration
    RegisterReal(ra,                       "ra",                       300.0, ScalarVariable::Type::TunableParameter);
    RegisterReal(time_to_start_iw_sampler, "time_to_start_iw_sampler", 100.0, ScalarVariable::Type::TunableParameter);
    RegisterInteger(max_inner_wp,          "max_inner_wp",             8,     ScalarVariable::Type::TunableParameter);
    RegisterInteger(max_sampled_inter_wp,  "max_sampled_inter_wp",     8,     ScalarVariable::Type::TunableParameter);
    RegisterReal(scope_angle_max_deg,      "scope_angle_max_deg",      45.0,  ScalarVariable::Type::TunableParameter);
    // fmicpp 0.5 registers Type::TunableParameter as fixed, so set it again here
    variability["ra"]                       = ScalarVariable::Variability::Tunable;
    variability["time_to_start_iw_sampler"] = ScalarVariable::Variability::Tunable;
    variability["max_inner_wp"]             = ScalarVariable::Variability::Tunable;
    variability["max_sampled_inter_wp"]     = ScalarVariable::Variability::Tunable;
    variability["scope_angle_max_deg"]      = ScalarVariable::Variability::Tunable;

    RegisterInput(north,           "north",           0.0);
    RegisterInput(east,            "east",            0.0);
    RegisterInput(scope_angle_deg, "scope_angle_deg", 0.0);
    RegisterInput(scope_length,    "scope_length",    2500.0);  // 2.5 km

    RegisterReal(wp_start_north, "wp_start_north", 0.0);
    RegisterReal(wp_start_east,  "wp_start_east",  0.0);
    RegisterReal(wp_start_speed, "wp_start_speed", 0.0);

    RegisterReal(wp_end_north, "wp_end_north", 0.0);
    RegisterReal(wp_end_east,  "wp_end_east",  0.0);
    RegisterReal(wp_end_speed, "wp_end_speed", 0.0);

    RegisterOutput(prev_wp_north,             "prev_wp_north",             0.0);
    RegisterOutput(prev_wp_east,              "prev_wp_east",              0.0);
    RegisterOutput(prev_wp_speed,             "prev_wp_speed",             0.0);
    RegisterOutput(next_wp_north,             "next_wp_north",             0.0);
    RegisterOutput(next_wp_east,              "next_wp_east",              0.0);
    RegisterOutput(next_wp_speed,             "next_wp_speed",             0.0);
    RegisterOutput(last_segment_active,       "last_segment_active",       false);
    RegisterOutput(reach_wp_end,              "reach_wp_end",              false);
    RegisterOutput(request_captain_intent,    "request_captain_intent",    false);
    RegisterOutput(insert_wp_now,             "insert_wp_now",             false);
    RegisterOutput(idx,                       "idx",                       0);
    RegisterOutput(next_wp_proj_north,        "next_wp_proj_north",        0.0);
    RegisterOutput(next_wp_proj_east,         "next_wp_proj_east",         0.0);
    RegisterOutput(accepted_sampled_inter_wp, "accepted_sampled_inter_wp", 0);

    // Messages
    RegisterOutput(messages, "messages", "-");
}

static bool _valid_triplet(double n, double e, double s)
{
    return std::isfinite(n) && std::isfinite(e) && std::isfinite(s);
}

// Position of a waypoint in a list. Throws if it is not there, like Python's list.index()
template<typename T>
static int _index(const std::vector<T>& list, const T& item)
{
    auto it = std::find(list.begin(), list.end(), item);
    if (it == list.end())
    {
        throw std::runtime_error("waypoint is not in list");
    }
    return static_cast<int>(it - list.begin());
}

template<typename T>
static bool _contains(const std::vector<T>& list, const T& item)
{
    return std::find(list.begin(), list.end(), item) != list.end();
}

static double deg2rad(double deg) { return deg * (std::numbers::pi / 180.0); }

// Python f"{x:.1f}"
static std::string _f1(double x)
{
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.1f", x);
    return buf;
}

// Python f"{x}" for a float, e.g. 2500.0 -> "2500.0", 1e-05 -> "1e-05"
static std::string _py_float_str(double x)
{
    if (std::isnan(x)) return "nan";
    if (std::isinf(x)) return x > 0 ? "inf" : "-inf";

    // Shortest digits that give x back, in scientific form: "d.ddde+XX"
    char buf[64];
    char* end = std::to_chars(buf, buf + sizeof(buf), x, std::chars_format::scientific).ptr;
    *end = '\0';
    int exponent = std::atoi(std::strchr(buf, 'e') + 1);
    if (exponent < -4 || exponent >= 16)
    {
        return buf;
    }

    // Otherwise the same digits without exponent, always with a decimal point
    end = std::to_chars(buf, buf + sizeof(buf), x, std::chars_format::fixed).ptr;
    std::string s(buf, end);
    if (s.find('.') == std::string::npos)
    {
        s += ".0";
    }
    return s;
}

void MODEL_IDENTIFIER::_build_trajectory()
{
    std::vector<Waypoint> traj;

    // Start
    if (_valid_triplet(wp_start_north, wp_start_east, wp_start_speed))
    {
        traj.push_back({wp_start_north, wp_start_east, wp_start_speed});
    }

    // End
    if (_valid_triplet(wp_end_north, wp_end_east, wp_end_speed))
    {
        traj.push_back({wp_end_north, wp_end_east, wp_end_speed});
    }

    _traj       = traj;
    idx         = 0;
    _traj_built = true;

    _traj_base                       = traj;
    _idx_in_segment                  = 1;
    accepted_sampled_inter_wp        = 0;
    _segment_idx                     = 1;
    _zeroeth_iw_inserted             = false;
    _temp_untraversed_segment_length = 0.0;

    // Initialize outputs if we have at least 2 points
    if (_traj.size() >= 2)
    {
        prev_wp_north = _traj[0][0];
        prev_wp_east  = _traj[0][1];
        prev_wp_speed = _traj[0][2];
        next_wp_north = _traj[1][0];
        next_wp_east  = _traj[1][1];
        next_wp_speed = _traj[1][2];
        last_segment_active = false;
    }
    else
    {
        // Not enough points
        prev_wp_north = prev_wp_east = prev_wp_speed = 0.0;
        next_wp_north = next_wp_east = next_wp_speed = 0.0;
        last_segment_active = false;
    }
}

double MODEL_IDENTIFIER::_dist2_to_next()
{
    if ((idx + 1) >= static_cast<int>(_traj.size()))
    {
        return INFINITY;
    }
    double dn = _traj[idx + 1][0] - north;
    double de = _traj[idx + 1][1] - east;
    return dn * dn + de * de;
}

void MODEL_IDENTIFIER::_get_route_segment_parameters(Waypoint& base_setpoint, Waypoint& head_setpoint,
                                                     double& segment_length, double& beta)
{
    // Get the base and the head setpoint
    base_setpoint = _traj_base.at(_segment_idx - 1);
    head_setpoint = _traj_base.at(_segment_idx);

    // Route segment components' length
    double d_segment_north = head_setpoint[0] - base_setpoint[0];
    double d_segment_east  = head_setpoint[1] - base_setpoint[1];

    // Compute the route segment length and angle from the North axis to the route segment line
    segment_length = std::hypot(d_segment_north, d_segment_east);
    beta           = std::atan2(d_segment_east, d_segment_north);
}

void MODEL_IDENTIFIER::_get_inter_wp_parameters(const Waypoint& inter_wp, const Waypoint& base_setpoint,
                                                const Waypoint& head_setpoint, double beta,
                                                double& untraversed_segment_length, Point& inter_wp_proj)
{
    double base_north = base_setpoint[0];
    double base_east  = base_setpoint[1];

    // Base to IW segment line components' length
    double d_base_to_inter_wp_north = inter_wp[0] - base_north;
    double d_base_to_inter_wp_east  = inter_wp[1] - base_east;

    // Base to IW segment line length
    double base_inter_wp_length = std::hypot(d_base_to_inter_wp_north, d_base_to_inter_wp_east);

    // Angle between the North axis to the IW segment line
    double beta_inter_wp = std::atan2(d_base_to_inter_wp_east, d_base_to_inter_wp_north);

    // Angle between the route segment line to the IW segment line
    double psi = beta_inter_wp - beta;

    // Traversed segment length
    double traversed_segment_length = base_inter_wp_length * std::cos(psi);

    // Projection point coordinate to the segment route
    double p_n = base_north + traversed_segment_length * std::cos(beta);
    double p_e = base_east  + traversed_segment_length * std::sin(beta);
    inter_wp_proj = {p_n, p_e};

    // Untraversed segment length
    untraversed_segment_length = std::hypot(head_setpoint[0] - p_n, head_setpoint[1] - p_e);
}

void MODEL_IDENTIFIER::_insert_inter_wp_to_traj(const Waypoint& inter_wp)
{
    // Get the route segment base_setpoint index in _traj
    Waypoint base_setpoint = _traj_base.at(_segment_idx - 1);
    int base_setpoint_idx  = _index(_traj, base_setpoint);

    // Identify the entry index for the given intermediate way point
    int entry_idx = base_setpoint_idx + _idx_in_segment;

    // Insert the intermediate waypoint to traj
    _traj.insert(_traj.begin() + entry_idx, inter_wp);

    // Increment the segment's waypoint index
    _idx_in_segment += 1;
}

void MODEL_IDENTIFIER::_get_zeroeth_intermediate_waypoint(Waypoint& inter_wp, Point& inter_wp_proj,
                                                          double& untraversed_segment_length)
{
    // Set the first ship position that triggers IS sampler as the 0th IS
    inter_wp = {north, east, next_wp_speed};  // IS's desired speed follows the segment route

    // Get the route segment parameters
    Waypoint base_setpoint, head_setpoint;
    double segment_length, beta;
    _get_route_segment_parameters(base_setpoint, head_setpoint, segment_length, beta);

    // Get the IW parameters
    _get_inter_wp_parameters(inter_wp, base_setpoint, head_setpoint, beta, untraversed_segment_length, inter_wp_proj);
}

void MODEL_IDENTIFIER::_get_intermediate_waypoint(double scope_angle_deg, double scope_length, Waypoint& inter_wp,
                                                  Point& inter_wp_proj, double& untraversed_segment_length)
{
    // Get route segment parameters
    Waypoint base_setpoint, head_setpoint;
    double segment_length, beta;
    _get_route_segment_parameters(base_setpoint, head_setpoint, segment_length, beta);

    // Get the previous waypoint as a base to scope the next intermediate waypoint
    int base_setpoint_idx = _index(_traj, base_setpoint);
    int prev_wp_idx       = base_setpoint_idx + _idx_in_segment - 1;  // -1 due to previous increment in _insert_inter_wp_to_traj
    Waypoint prev_wp      = _traj.at(prev_wp_idx);

    // Scope the next intermediate waypoint given the input scope angle
    double beta_inter_wp = beta + deg2rad(scope_angle_deg);
    double n_inter_wp    = prev_wp[0] + scope_length * std::cos(beta_inter_wp);
    double e_inter_wp    = prev_wp[1] + scope_length * std::sin(beta_inter_wp);
    inter_wp = {n_inter_wp, e_inter_wp, next_wp_speed};

    // Get the IW parameters
    _get_inter_wp_parameters(inter_wp, base_setpoint, head_setpoint, beta, untraversed_segment_length, inter_wp_proj);
}

void MODEL_IDENTIFIER::_save_intermediate_waypoint(const Waypoint& inter_wp, const Point& inter_wp_proj)
{
    _insert_inter_wp_to_traj(inter_wp);

    // Store output
    next_wp_proj_north = inter_wp_proj[0];
    next_wp_proj_east  = inter_wp_proj[1];
}

void MODEL_IDENTIFIER::_get_prev_and_next_waypoint()
{
    // Update prev/next using new index
    int n = static_cast<int>(_traj.size());
    Waypoint p = (idx + 1) < n ? _traj[idx]     : _traj[n - 2];
    Waypoint q = (idx + 1) < n ? _traj[idx + 1] : _traj[n - 1];

    prev_wp_north = p[0];
    prev_wp_east  = p[1];
    prev_wp_speed = p[2];
    next_wp_north = q[0];
    next_wp_east  = q[1];
    next_wp_speed = q[2];
}

void MODEL_IDENTIFIER::do_step(double current_time, double step_size)
{
    try
    {
        // Build once
        if (!_traj_built)
        {
            _build_trajectory();
        }

        // If not enough waypoints, nothing to do
        if (_traj.size() < 2)
        {
            return;
        }

        // Check if the ship enters radius of acceptance
        double ra2       = ra * ra;
        bool entering_ra = _dist2_to_next() <= ra2;

        // Next-point status
        int n                 = static_cast<int>(_traj.size());
        bool next_wp_exists   = (idx + 1) < n;
        bool next_wp_is_end   = next_wp_exists && (_traj[idx + 1] == _traj_base.back());
        bool next_wp_is_inter = next_wp_exists && !_contains(_traj_base, _traj[idx + 1]);

        // Last segment flag
        last_segment_active = (idx == n - 2);

        // List of triggers
        bool within_sampling_max_count = accepted_sampled_inter_wp < max_sampled_inter_wp;
        bool iw_sampler_active         = current_time >= time_to_start_iw_sampler;
        bool place_zeroeth_inter_wp    = within_sampling_max_count && iw_sampler_active && !_zeroeth_iw_inserted;
        bool enable_inter_wp_sampling  = within_sampling_max_count && iw_sampler_active && _zeroeth_iw_inserted
                                         && entering_ra && next_wp_is_inter;

        insert_wp_now = false;

        // ======================================================================================
        // First Condition: Handle the captain intent request
        // ======================================================================================
        if (request_captain_intent)
        {
            // If the scope length is longer than the untraversed segment length (saved before),
            // the new waypoint would be beyond the end waypoint
            bool valid_insertion = _temp_untraversed_segment_length >= scope_length;

            // Advance the trajectory, not saving the waypoint and temporarily pend
            // the captain intent request while awaiting for the next waypoint RoA visit
            if (!valid_insertion)
            {
                idx += 1;
                _get_prev_and_next_waypoint();

                request_captain_intent = false;

                messages = "New intermediate waypoint is sampled beyond the end waypoint";
                return;
            }

            // Compute the intermediate waypoint
            Waypoint inter_wp;
            Point inter_wp_proj;
            double untraversed_segment_length;
            _get_intermediate_waypoint(scope_angle_deg, scope_length, inter_wp, inter_wp_proj, untraversed_segment_length);

            // Advance the trajectory
            _save_intermediate_waypoint(inter_wp, inter_wp_proj);
            idx += 1;
            _get_prev_and_next_waypoint();

            std::string sub_messages = "Captain intent " + std::to_string(accepted_sampled_inter_wp + 1)
                                       + " using scope angle of " + _f1(scope_angle_deg)
                                       + " degrees and scope length of " + _py_float_str(scope_length);

            // Update the temporary untraversed segment length variable
            _temp_untraversed_segment_length = untraversed_segment_length;

            // Then turn off the captain intent request after receiving it
            request_captain_intent = false;

            // Increment the sampling count after the sampling
            accepted_sampled_inter_wp += 1;

            // Set the internal function to insert the wp now for recording
            insert_wp_now = true;

            messages = "Captain intent request handling: " + sub_messages;
            return;
        }

        // ======================================================================================
        // Second Condition: Zeroeth intermediate waypoint placement + request first captain intent
        // ======================================================================================
        if (place_zeroeth_inter_wp)
        {
            // Update the actual segment index. Inside WP trigger does not immediately triggers at the first waypoint
            Waypoint prev_wp = {prev_wp_north, prev_wp_east, prev_wp_speed};
            _segment_idx     = _index(_traj_base, prev_wp) + 1;

            // Place the zeroeth then save intermediate waypoint
            Waypoint inter_wp;
            Point inter_wp_proj;
            double untraversed_segment_length;
            _get_zeroeth_intermediate_waypoint(inter_wp, inter_wp_proj, untraversed_segment_length);
            _save_intermediate_waypoint(inter_wp, inter_wp_proj);

            // Stay at the current trajectory
            _get_prev_and_next_waypoint();

            // Save the temporary untraversed_segment_length
            _temp_untraversed_segment_length = untraversed_segment_length;

            // Request the scope angle for the next step
            request_captain_intent = true;
            _zeroeth_iw_inserted   = true;

            // Set the internal function to insert the wp now for recording
            insert_wp_now = true;

            messages = "Zeroeth IW Placement at (" + _f1(next_wp_north) + "," + _f1(next_wp_east)
                       + ") + First captain intent request.";
            return;
        }

        // ======================================================================================
        // Third Condition: Request second and onwards captain intent
        // ======================================================================================
        if (enable_inter_wp_sampling)
        {
            // Request the scope angle for the next step
            request_captain_intent = true;

            messages = "Entering waypoint (" + _f1(next_wp_north) + "," + _f1(next_wp_east)
                       + ") RoA -> next captain intent request";
            return;
        }

        // ======================================================================================
        // Fourth Condition: End Condition
        // ======================================================================================
        if (entering_ra && next_wp_is_end)
        {
            reach_wp_end  = true;
            prev_wp_speed = 0.0;
            next_wp_speed = 0.0;

            messages = "Reaching the final waypoint (" + _f1(next_wp_north) + "," + _f1(next_wp_east) + ")";
            return;
        }

        // ======================================================================================
        // Fifth Condition: Normal Progression
        // ======================================================================================
        if (entering_ra)
        {
            // Advance the trajectory
            idx += 1;
            _get_prev_and_next_waypoint();

            messages = "Entering waypoint (" + _f1(prev_wp_north) + "," + _f1(prev_wp_east) + ") RoA";
            return;
        }

        messages = "Try to converge to segment (" + _f1(prev_wp_north) + "," + _f1(prev_wp_east) + ")->("
                   + _f1(next_wp_north) + "," + _f1(next_wp_east) + ")";
    }
    catch (const std::exception& e)
    {
        // Keep host alive; do not crash co-sim
        std::cout << "[MissionManagerAST] Exception t=" << current_time << ", dt=" << step_size << ": " << e.what() << std::endl;

        // Freeze dynamics safely (keep last state/outputs)
        last_segment_active    = false;
        reach_wp_end           = false;
        request_captain_intent = false;
        insert_wp_now          = false;
    }
}

void MODEL_IDENTIFIER::DoStep(const double globalTime)
{
    do_step(time, globalTime - time);
    SystemBaseWithSolvers::DoStep(globalTime);
}
