#include "modelName.h"
#include "systemBase.h"
#include SYSTEM_HEADER
#include "guid.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

MODEL_IDENTIFIER::MODEL_IDENTIFIER(std::shared_ptr<LoggerBase> _logger, std::string _location)
    : SystemBaseWithSolvers(_logger, _location)
{
    author      = "Andreas R.G. Sitorus, Etienne Purcell";
    description = "Mission Manager (index-based, deterministic)";
    version     = MODEL_VERSION;
    name        = MODELNAME;
    identifier  = MODEL_IDENTIFIER_STR;
    guid        = fmuGUID;

    // No built-in solver: do_step() does the work
    solverType = "None";
    printInfo  = false;

    // Internal (hidden, but saved with the FMU state)
    RegisterInternal(_idx,        "_idx",        0);
    RegisterInternal(_traj_built, "_traj_built", false);

    // Registration
    RegisterReal(ra, "ra", 300.0, ScalarVariable::Type::TunableParameter);
    RegisterInteger(max_inner_wp, "max_inner_wp", 8, ScalarVariable::Type::TunableParameter);
    // fmicpp 0.5 registers Type::TunableParameter as fixed, so set it again here
    variability["ra"]           = ScalarVariable::Variability::Tunable;
    variability["max_inner_wp"] = ScalarVariable::Variability::Tunable;

    RegisterInput(north, "north", 0.0);
    RegisterInput(east,  "east",  0.0);

    RegisterReal(wp_start_north, "wp_start_north", 0.0);
    RegisterReal(wp_start_east,  "wp_start_east",  0.0);
    RegisterReal(wp_start_speed, "wp_start_speed", 0.0);

    for (int i = 1; i < 9; i++)
    {
        RegisterReal(wp_north[i - 1], "wp_" + std::to_string(i) + "_north", 0.0);
        RegisterReal(wp_east[i - 1],  "wp_" + std::to_string(i) + "_east",  0.0);
        RegisterReal(wp_speed[i - 1], "wp_" + std::to_string(i) + "_speed", 0.0);
    }

    RegisterReal(wp_end_north, "wp_end_north", 0.0);
    RegisterReal(wp_end_east,  "wp_end_east",  0.0);
    RegisterReal(wp_end_speed, "wp_end_speed", 0.0);

    RegisterOutput(prev_wp_north, "prev_wp_north", 0.0);
    RegisterOutput(prev_wp_east,  "prev_wp_east",  0.0);
    RegisterOutput(prev_wp_speed, "prev_wp_speed", 0.0);

    RegisterOutput(next_wp_north, "next_wp_north", 0.0);
    RegisterOutput(next_wp_east,  "next_wp_east",  0.0);
    RegisterOutput(next_wp_speed, "next_wp_speed", 0.0);

    RegisterOutput(last_wp_active, "last_wp_active", false);

    RegisterOutput(reach_wp_end, "reach_wp_end", false);
}

static bool _valid_triplet(double n, double e, double s)
{
    return std::isfinite(n) && std::isfinite(e) && std::isfinite(s);
}

void MODEL_IDENTIFIER::_add_to_traj(double n, double e, double s)
{
    _traj_north[_traj_len] = n;
    _traj_east[_traj_len]  = e;
    _traj_speed[_traj_len] = s;
    _traj_len += 1;
}

void MODEL_IDENTIFIER::_build_trajectory()
{
    _traj_len = 0;

    // Start
    if (_valid_triplet(wp_start_north, wp_start_east, wp_start_speed))
    {
        _add_to_traj(wp_start_north, wp_start_east, wp_start_speed);
    }

    // Intermediate (1..max_inner_wp)
    int m = std::max(0, std::min(max_inner_wp, 8));
    for (int i = 1; i < m + 1; i++)
    {
        double n = wp_north[i - 1];
        double e = wp_east[i - 1];
        double s = wp_speed[i - 1];
        if (_valid_triplet(n, e, s))
        {
            _add_to_traj(n, e, s);
        }
    }

    // End
    if (_valid_triplet(wp_end_north, wp_end_east, wp_end_speed))
    {
        _add_to_traj(wp_end_north, wp_end_east, wp_end_speed);
    }

    _idx = 0;
    _traj_built = true;

    // Initialize outputs if we have at least 2 points
    if (_traj_len >= 2)
    {
        prev_wp_north = _traj_north[0];
        prev_wp_east  = _traj_east[0];
        prev_wp_speed = _traj_speed[0];
        next_wp_north = _traj_north[1];
        next_wp_east  = _traj_east[1];
        next_wp_speed = _traj_speed[1];
        last_wp_active = false;
    }
    else
    {
        // Not enough points
        prev_wp_north = prev_wp_east = prev_wp_speed = 0.0;
        next_wp_north = next_wp_east = next_wp_speed = 0.0;
        last_wp_active = false;
    }
}

double MODEL_IDENTIFIER::_dist2_to_next()
{
    double dn = _traj_north[_idx + 1] - north;
    double de = _traj_east[_idx + 1] - east;
    return dn * dn + de * de;
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
        if (_traj_len < 2)
        {
            return;
        }

        double ra2 = ra * ra;

        // If we're already on last segment, latch and keep outputs fixed
        // while keep checking if the ship reaches the end point.
        // Once reach_wp_end is true, it stays true.
        if (_idx >= _traj_len - 2)
        {
            last_wp_active = true;

            // Check if the ship reaches the end point
            if (_dist2_to_next() <= ra2)
            {
                last_wp_active = true;
                reach_wp_end   = true;

                // Set the speed set point to 0 (Stopping)
                prev_wp_speed = 0.0;
                next_wp_speed = 0.0;
            }

            return;
        }

        // Switch ONLY when close to the NEXT waypoint
        if (_dist2_to_next() <= ra2)
        {
            _idx += 1;

            // Update prev/next using new index
            int q = (_idx + 1) < _traj_len ? _idx + 1 : _traj_len - 1;

            prev_wp_north = _traj_north[_idx];
            prev_wp_east  = _traj_east[_idx];
            prev_wp_speed = _traj_speed[_idx];
            next_wp_north = _traj_north[q];
            next_wp_east  = _traj_east[q];
            next_wp_speed = _traj_speed[q];
        }
    }
    catch (const std::exception& e)
    {
        // Keep host alive; do not crash co-sim
        std::cout << "[MissionManager] Exception t=" << current_time << ", dt=" << step_size << ": " << e.what() << std::endl;

        prev_wp_north = 0.0;
        prev_wp_east  = 0.0;
        prev_wp_speed = 0.0;

        next_wp_north = 0.0;
        next_wp_east  = 0.0;
        next_wp_speed = 0.0;

        last_wp_active = false;

        reach_wp_end = false;
    }
}

void MODEL_IDENTIFIER::DoStep(const double globalTime)
{
    do_step(time, globalTime - time);
    SystemBaseWithSolvers::DoStep(globalTime);
}
