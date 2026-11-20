#include "modelName.h"
#include "systemBase.h"
#include SYSTEM_HEADER
#include "guid.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <string>

MODEL_IDENTIFIER::MODEL_IDENTIFIER(std::shared_ptr<LoggerBase> _logger, std::string _location)
    : SystemBaseWithSolvers(_logger, _location)
{
    author      = "Andreas R.G. Sitorus, Etienne Purcell";
    description = "Simple Collision Avoidance Algorithm C++ FMU Implementation";
    version     = MODEL_VERSION;
    name        = MODELNAME;
    identifier  = MODEL_IDENTIFIER_STR;
    guid        = fmuGUID;

    // No built-in solver: do_step() does the work
    solverType = "None";
    printInfo  = false;

    // Registration
    // Parameters
    RegisterReal(throttle_scale_factor, "throttle_scale_factor", 0.5,   ScalarVariable::Type::TunableParameter);
    RegisterReal(rud_ang_increment_deg, "rud_ang_increment_deg", 15.0,  ScalarVariable::Type::TunableParameter);
    RegisterReal(danger_zone_radius,    "danger_zone_radius",    926.0, ScalarVariable::Type::TunableParameter);
    RegisterReal(collision_zone_radius, "collision_zone_radius", 100.0, ScalarVariable::Type::TunableParameter);
    RegisterInteger(max_target_ship_count, "max_target_ship_count", 0);
    RegisterReal(hold_time,             "hold_time",             600.0, ScalarVariable::Type::TunableParameter);
    RegisterReal(T_lookahead,           "T_lookahead",           900.0, ScalarVariable::Type::TunableParameter);
    // fmicpp 0.5 registers Type::TunableParameter as fixed, so set it again here
    variability["throttle_scale_factor"] = ScalarVariable::Variability::Tunable;
    variability["rud_ang_increment_deg"] = ScalarVariable::Variability::Tunable;
    variability["danger_zone_radius"]    = ScalarVariable::Variability::Tunable;
    variability["collision_zone_radius"] = ScalarVariable::Variability::Tunable;
    variability["hold_time"]             = ScalarVariable::Variability::Tunable;
    variability["T_lookahead"]           = ScalarVariable::Variability::Tunable;

    // Input
    RegisterInput(throttle_cmd,     "throttle_cmd",     0.0);
    RegisterInput(rudder_angle_deg, "rudder_angle_deg", 0.0);

    RegisterInput(own_north,          "own_north",          0.0);
    RegisterInput(own_east,           "own_east",           0.0);
    RegisterInput(own_yaw_angle,      "own_yaw_angle",      0.0);
    RegisterInput(own_measured_speed, "own_measured_speed", 0.0);

    // For each ship, set the initial position to be far away, facing north, and static
    for (int i = 1; i < 4; i++)
    {
        std::string tar = "tar_" + std::to_string(i);
        RegisterInput(tar_north[i - 1],          tar + "_north",          1e9);
        RegisterInput(tar_east[i - 1],           tar + "_east",           1e9);
        RegisterInput(tar_yaw_angle[i - 1],      tar + "_yaw_angle",      0.0);
        RegisterInput(tar_measured_speed[i - 1], tar + "_measured_speed", 0.0);
    }

    // Output
    RegisterOutput(new_throttle_cmd,        "new_throttle_cmd",        0.0);
    RegisterOutput(new_rudder_angle_deg,    "new_rudder_angle_deg",    0.0);
    RegisterOutput(colav_rud_ang_increment, "colav_rud_ang_increment", 0.0);
    RegisterOutput(ship_priority_idx,       "ship_priority_idx",       -1);
    RegisterOutput(colav_active,            "colav_active",            false);
    RegisterOutput(ship_collision,          "ship_collision",          false);

    for (int i = 1; i < 4; i++)
    {
        std::string tar = "_own_to_tar_" + std::to_string(i);
        RegisterOutput(beta_own_to_tar[i - 1], "beta" + tar, 0.0);
        RegisterOutput(tcpa_own_to_tar[i - 1], "tcpa" + tar, 0.0);
        RegisterOutput(dcpa_own_to_tar[i - 1], "dcpa" + tar, 0.0);
        RegisterOutput(dist_own_to_tar[i - 1], "dist" + tar, 0.0);
        RegisterOutput(rr_own_to_tar[i - 1],   "rr" + tar,   0.0);
    }

    // Internal Variable (hidden, but saved with the FMU state)
    RegisterInternal(prioritize_one_target, "prioritize_one_target", false);
    RegisterInternal(held_target_idx,       "held_target_idx",       0);
    RegisterInternal(timer,                 "timer",                 0.0);
    RegisterInternal(colav_rudder_sign,     "colav_rudder_sign",     -1.0);
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

// Same as np.clip for one value
static double clip(double val, double low, double hi)
{
    return std::min(std::max(val, low), hi);
}

// Same as np.dot for two 2D vectors
static double dot2(const double a[2], const double b[2])
{
    return a[0] * b[0] + a[1] * b[1];
}

// Same as np.sign
static double sign(double a)
{
    return (a > 0.0) ? 1.0 : ((a < 0.0) ? -1.0 : 0.0);
}

void MODEL_IDENTIFIER::do_step(double current_time, double step_size)
{
    try
    {
        // To take into account issue with all-zero initial value,
        // Turn off the COLAV during the very first time step
        if (current_time < step_size)
        {
            new_throttle_cmd     = 0.0;
            new_rudder_angle_deg = 0.0;
            colav_active         = false;
            ship_collision       = false;

            for (int i = 0; i < 3; i++)
            {
                beta_own_to_tar[i] = 0.0;
                tcpa_own_to_tar[i] = 0.0;
                dcpa_own_to_tar[i] = 0.0;
                dist_own_to_tar[i] = 0.0;
                rr_own_to_tar[i]   = 0.0;
            }

            return;
        }

        // INITIATE
        double p_own[2] = {own_north, own_east};
        double v_own[2] = {own_measured_speed * std::cos(own_yaw_angle), own_measured_speed * std::sin(own_yaw_angle)};
        double p_tar_list[3][2];
        double v_tar_list[3][2];
        double beta_list[3];
        double tcpa_list[3];
        double dcpa_list[3];
        double dist_list[3];
        double range_rate_list[3];

        int c = max_target_ship_count;
        c = std::max(0, std::min(c, 3));

        for (int i = 0; i < c; i++)
        {
            p_tar_list[i][0] = tar_north[i];
            p_tar_list[i][1] = tar_east[i];
            v_tar_list[i][0] = tar_measured_speed[i] * std::cos(tar_yaw_angle[i]);
            v_tar_list[i][1] = tar_measured_speed[i] * std::sin(tar_yaw_angle[i]);
        }

        // Compute Relative Bearing (Beta), TCPA, and DCPA, Range Rate
        for (int i = 0; i < c; i++)
        {
            // Relative distance
            double r[2] = {p_tar_list[i][0] - p_own[0], p_tar_list[i][1] - p_own[1]};

            // RELATIVE VELOCITY
            double v[2] = {v_tar_list[i][0] - v_own[0], v_tar_list[i][1] - v_own[1]};

            // Relative Bearing
            double theta = std::atan2(r[1], r[0]);
            double beta  = wrap_to_pi(theta - own_yaw_angle);
            beta_own_to_tar[i] = beta;
            beta_list[i] = beta;

            // TCPA and DCPA
            double v_norm_sq = dot2(v, v);

            double tcpa, dcpa;
            // If the squared norm of the speed is close to 0
            if (v_norm_sq < 1e-6)
            {
                tcpa = std::numeric_limits<double>::quiet_NaN();  // Both ships are stationary to each other, signify as nan
                dcpa = std::sqrt(dot2(r, r));
            }
            else
            {
                tcpa = -dot2(r, v) / v_norm_sq;
                double r_cpa[2] = {r[0] + v[0] * tcpa, r[1] + v[1] * tcpa};
                dcpa = std::sqrt(dot2(r_cpa, r_cpa));
            }

            tcpa_own_to_tar[i] = tcpa;
            dcpa_own_to_tar[i] = dcpa;

            tcpa_list[i] = tcpa;
            dcpa_list[i] = dcpa;

            // RANGE RATE
            double dist = std::sqrt(dot2(r, r));
            dist_list[i] = dist;
            dist_own_to_tar[i] = dist;
            double range_rate;
            if (dist < 1e-6)
            {
                range_rate = 0.0;
            }
            else
            {
                double r_hat[2] = {r[0] / dist, r[1] / dist};
                range_rate = dot2(r_hat, v);
            }
            rr_own_to_tar[i] = range_rate;
            range_rate_list[i] = range_rate;
        }

        // COLAV DECISION
        // Set the collision status
        ship_collision = false;  // OUTPUT
        for (int i = 0; i < c; i++)
        {
            if (dist_list[i] <= collision_zone_radius)
            {
                ship_collision = true;
            }
        }

        if (ship_collision)
        {
            new_throttle_cmd     = clip(throttle_cmd, 0.0, 1.0);  // OUTPUT, clamp after scaling
            new_rudder_angle_deg = rudder_angle_deg;              // OUTPUT

            return;
        }

        // Score the threat level for each target ship
        double candidate_score[3];
        int candidate_idx[3];
        int n_candidates = 0;
        for (int i = 0; i < c; i++)
        {
            double tcpa = tcpa_list[i];
            double dcpa = dcpa_list[i];
            double rr   = range_rate_list[i];

            // Colav status
            // Active if there exists target with (in order):
            // dcpa < danger_zone_radius -> If it going to violate the safety region
            // tcpa > 0                  -> If collision will happen in the future
            // tcpa < T_lookahead        -> If collision will happen soon
            // rr   < 0 (ideally)        -> Ideally is closing in

            // Initial colav status
            colav_active = false;  // OUTPUT

            if (dcpa > danger_zone_radius)
                continue;
            if (tcpa <= 0)
                continue;
            if (tcpa > T_lookahead)
                continue;
            if (rr >= 0)
                continue;

            // Set colav is active if it passed through the logic gate
            colav_active = true;  // OUTPUT

            // Risk score
            double score = (1.0 / (tcpa + 1e-3) + (1.0 / (dcpa + 1e-3)) + 0.1 * (-rr));
            candidate_score[n_candidates] = score;
            candidate_idx[n_candidates]   = i;
            n_candidates += 1;
        }

        // Index Hold Logic
        auto is_still_candidate = [&](int i)
        {
            double tcpa = tcpa_list[i];
            return (tcpa > 0.0) && (range_rate_list[i] < 0.0) && (dcpa_list[i] <= danger_zone_radius);
        };

        // If the prioritized target is no longer a threat, (or the held target index is invalid) release the hold
        if (prioritize_one_target)
        {
            if ((held_target_idx < 0) || (held_target_idx >= c) || (!is_still_candidate(held_target_idx)))
            {
                prioritize_one_target = false;
                timer = 0.0;
            }
        }

        // Get the maximum score, then respons, else do nothing
        if (n_candidates > 0)
        {
            // Same as Python max(candidates, key=score): the first candidate with the highest score
            int best = 0;
            for (int k = 1; k < n_candidates; k++)
            {
                if (candidate_score[k] > candidate_score[best])
                {
                    best = k;
                }
            }
            int idx = candidate_idx[best];

            // Prioritize one_target if multiple ship in vicinity (and not yet prioritizing one target)
            // Start tracking the target
            if (n_candidates > 1 && (!prioritize_one_target))
            {
                held_target_idx = idx;
                prioritize_one_target = true;  // PRIORITIZING ONLY HAPPEN WHEN WE HAVE >1 CANDIDATES
                timer = 0.0;

                // Also held the colav rudder sign
                double beta = beta_list[held_target_idx];
                colav_rudder_sign = (std::abs(beta) > 1e-6) ? -sign(beta) : -1.0;
            }

            // If prioritize one target use held_target_idx, else use idx
            int use_idx = prioritize_one_target ? held_target_idx : idx;

            // Outputs prioritized target ship index
            ship_priority_idx = use_idx;

            // Accrue the rudder angle increment
            colav_rud_ang_increment += colav_rudder_sign * rud_ang_increment_deg;  // OUTPUT

            // OUTPUTS
            new_throttle_cmd     = clip(throttle_scale_factor * throttle_cmd, 0.0, 1.0);  // clamp after scaling
            new_rudder_angle_deg = rudder_angle_deg + colav_rud_ang_increment;

            if (prioritize_one_target)
            {
                timer += step_size;
                if (timer >= hold_time)
                {
                    prioritize_one_target = false;
                    timer = 0.0;
                }
            }
        }
        else
        {
            // OUTPUTS
            new_throttle_cmd        = clip(throttle_cmd, 0.0, 1.0);  // OUTPUT, clamp after scaling
            new_rudder_angle_deg    = rudder_angle_deg;              // OUTPUT
            colav_rud_ang_increment = 0.0;                           // OUTPUT

            // No target ship is prioritized
            ship_priority_idx = -1;
        }
    }
    catch (const std::exception& e)
    {
        // IMPORTANT: do not crash host
        std::cout << "[SimpleCollisionAvoidance] ERROR t=" << current_time << " dt=" << step_size << ": " << e.what() << std::endl;

        new_throttle_cmd        = clip(throttle_cmd, 0.0, 1.0);  // clamp after scaling
        new_rudder_angle_deg    = rudder_angle_deg;
        colav_rud_ang_increment = 0.0;  // OUTPUT
        colav_active            = false;
        ship_collision          = false;
    }
}

void MODEL_IDENTIFIER::DoStep(const double globalTime)
{
    do_step(time, globalTime - time);
    SystemBaseWithSolvers::DoStep(globalTime);
}
