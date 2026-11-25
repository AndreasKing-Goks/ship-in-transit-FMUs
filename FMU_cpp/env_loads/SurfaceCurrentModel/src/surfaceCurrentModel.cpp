#include "modelName.h"
#include "systemBase.h"
#include SYSTEM_HEADER
#include "guid.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <numbers>

MODEL_IDENTIFIER::MODEL_IDENTIFIER(std::shared_ptr<LoggerBase> _logger, std::string _location)
    : SystemBaseWithSolvers(_logger, _location)
{
    author      = "Andreas R.G. Sitorus, Etienne Purcell";
    description = "Surface Current Model C++ FMU Implementation";
    version     = MODEL_VERSION;
    name        = MODELNAME;
    identifier  = MODEL_IDENTIFIER_STR;
    guid        = fmuGUID;

    // No built-in solver: do_step() does the work
    solverType = "None";
    printInfo  = false;

    // Registration
    // Fixed parameters
    RegisterInteger(seed, "seed", 0);

    RegisterReal(initial_current_speed,            "initial_current_speed",            0.0);
    RegisterReal(current_speed_decay_rate,         "current_speed_decay_rate",         0.0);
    RegisterReal(current_speed_standard_deviation, "current_speed_standard_deviation", 0.0);

    RegisterReal(initial_current_direction_deg,            "initial_current_direction_deg",            0.0);
    RegisterReal(current_direction_deg_decay_rate,         "current_direction_deg_decay_rate",         0.0);
    RegisterReal(current_direction_deg_standard_deviation, "current_direction_deg_standard_deviation", 0.0);

    RegisterBoolean(clip_speed_nonnegative, "clip_speed_nonnegative", true);
    RegisterBoolean(fail_outputs_zero,      "fail_outputs_zero",      true);

    // Inputs (recommended for RL-driven mid-sim changes)
    RegisterInput(mean_current_speed,         "mean_current_speed",         0.0);
    RegisterInput(mean_current_direction_deg, "mean_current_direction_deg", 0.0);

    // Outputs
    RegisterOutput(current_speed,         "current_speed",         0.0);
    RegisterOutput(current_direction_rad, "current_direction_rad", 0.0);
    RegisterOutput(current_direction_deg, "current_direction_deg", 0.0);
    RegisterOutput(current_valid,         "current_valid",         true);

    // Internal state (hidden, but saved with the FMU state; the random generator is not saved)
    RegisterInternal(pre_compute, "pre_compute", false);
    RegisterInternal(vel,         "vel",         0.0);
    RegisterInternal(dir,         "dir",         0.0);
}

// =========================
// Helpers
// =========================

// Same as Python: (a + pi) % (2*pi) - pi.
// Python's % keeps the sign of 2*pi, so a negative result from fmod is moved up by 2*pi.
static double wrap_pi(double a)
{
    const double pi = std::numbers::pi;
    double m = std::fmod(a + pi, 2.0 * pi);
    if (m < 0.0)
    {
        m += 2.0 * pi;
    }
    return m - pi;
}

// Same as np.deg2rad and np.rad2deg
static double deg2rad(double deg) { return deg * (std::numbers::pi / 180.0); }
static double rad2deg(double rad) { return rad * (180.0 / std::numbers::pi); }

// Normal random number N(0,1) (Box-Muller). Written out, because std::normal_distribution
// gives different numbers on different compilers.
static double random_normal(std::mt19937_64& rng)
{
    double u1 = (rng() >> 11) * (1.0 / 9007199254740992.0);  // uniform in [0, 1)
    double u2 = (rng() >> 11) * (1.0 / 9007199254740992.0);
    return std::sqrt(-2.0 * std::log(1.0 - u1)) * std::cos(2.0 * std::numbers::pi * u2);
}

void MODEL_IDENTIFIER::pre_compute_initial_parameters()
{
    // RNG
    rng.seed(static_cast<std::uint64_t>(seed));

    // State
    vel = initial_current_speed;
    dir = wrap_pi(deg2rad(initial_current_direction_deg));

    // OU parameters
    mu_vel = current_speed_decay_rate;
    mu_dir = current_direction_deg_decay_rate;

    sigma_vel = current_speed_standard_deviation;
    sigma_dir = deg2rad(current_direction_deg_standard_deviation);
}

// Exact OU step (stable for variable dt)
// m -> mean value
// X_{k+1} = m + (X_k - m) * exp(-mu dt) + sigma * sqrt((1-exp(-2 mu dt))/(2 mu)) * N(0,1)
double MODEL_IDENTIFIER::ou_exact_step(double x, double m, double mu, double sigma, double dt)
{
    if (dt <= 0.0)
    {
        return x;  // nothing to do
    }

    if (mu <= 0.0)
    {
        // Pure random walk around mean with drift disabled:
        // x_{k+1} = x_k + sigma*sqrt(dt)*N(0,1)  (no reversion)
        return x + sigma * std::sqrt(dt) * random_normal(rng);
    }

    double a   = std::exp(-mu * dt);
    double var = (1.0 - std::exp(-2.0 * mu * dt)) / (2.0 * mu);
    return m + (x - m) * a + sigma * std::sqrt(var) * random_normal(rng);
}

// =========================
// Main stepping
// =========================
// Mean-driven OU:
//   dV   = -mu_vel (V - mean_speed) dt + sigma_vel dW
//   dpsi = -mu_dir wrap(psi - mean_dir) dt + sigma_dir dW
void MODEL_IDENTIFIER::do_step(double current_time, double step_size)
{
    try
    {
        if (!pre_compute)
        {
            pre_compute_initial_parameters();
            pre_compute = true;
        }

        double dt = step_size;
        if (dt <= 0.0)
        {
            // Keep outputs/state unchanged; still "valid"
            current_valid = true;
            return;
        }

        // Read inputs (master can keep them constant for many steps and change anytime)
        double mean_speed = mean_current_speed;
        double mean_dir   = deg2rad(mean_current_direction_deg);

        // If you truly want speed mean always nonnegative:
        mean_speed = std::abs(mean_speed);

        // --- Speed OU to mean ---
        vel = ou_exact_step(vel, mean_speed, mu_vel, sigma_vel, dt);
        if (clip_speed_nonnegative)
        {
            vel = std::max(0.0, vel);
        }

        // --- Direction OU on circle (revert using shortest angular error) ---
        // We update psi by applying OU on the *error* around the mean.
        mean_dir   = wrap_pi(mean_dir);
        double err = wrap_pi(dir - mean_dir);

        // OU exact step on error toward 0
        double err_new = ou_exact_step(err, 0.0, mu_dir, sigma_dir, dt);
        dir = wrap_pi(mean_dir + err_new);

        // outputs
        current_speed         = vel;
        current_direction_rad = dir;
        current_direction_deg = rad2deg(dir);
        current_valid         = true;
    }
    catch (const std::exception& e)
    {
        std::cout << "[SurfaceCurrentModel] Exception t=" << current_time << ", dt=" << step_size << ": " << e.what() << std::endl;

        current_valid = false;
        if (fail_outputs_zero)
        {
            current_speed         = 0.0;
            current_direction_rad = 0.0;
            current_direction_deg = 0.0;
        }
        // else: hold last outputs (do nothing)
    }
}

void MODEL_IDENTIFIER::DoStep(const double globalTime)
{
    do_step(time, globalTime - time);
    SystemBaseWithSolvers::DoStep(globalTime);
}
