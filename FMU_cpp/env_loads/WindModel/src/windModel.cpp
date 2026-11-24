#include "modelName.h"
#include "systemBase.h"
#include SYSTEM_HEADER
#include "guid.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <numbers>
#include <stdexcept>

MODEL_IDENTIFIER::MODEL_IDENTIFIER(std::shared_ptr<LoggerBase> _logger, std::string _location)
    : SystemBaseWithSolvers(_logger, _location)
{
    author      = "Andreas R.G. Sitorus, Etienne Purcell";
    description = "Wind Model C++ FMU Implementation";
    version     = MODEL_VERSION;
    name        = MODELNAME;
    identifier  = MODEL_IDENTIFIER_STR;
    guid        = fmuGUID;

    // No built-in solver: do_step() does the work
    solverType = "None";
    printInfo  = false;

    // Registration
    // Random seed
    RegisterInteger(seed, "seed", 0);

    // Mean wind speed parameters
    RegisterReal(initial_mean_wind_speed,            "initial_mean_wind_speed",            0.0);
    RegisterReal(mean_wind_speed_decay_rate,         "mean_wind_speed_decay_rate",         0.0);
    RegisterReal(mean_wind_speed_standard_deviation, "mean_wind_speed_standard_deviation", 0.0);

    // Wind direction parameters
    RegisterReal(initial_wind_direction_deg,            "initial_wind_direction_deg",            0.0);
    RegisterReal(wind_direction_deg_decay_rate,         "wind_direction_deg_decay_rate",         0.0);
    RegisterReal(wind_direction_deg_standard_deviation, "wind_direction_deg_standard_deviation", 0.0);

    // Wind constraints and gust parameters
    RegisterReal(minimum_mean_wind_speed,     "minimum_mean_wind_speed",     0.0);
    RegisterReal(maximum_mean_wind_speed,     "maximum_mean_wind_speed",     0.0);
    RegisterReal(minimum_wind_gust_frequency, "minimum_wind_gust_frequency", 0.0);
    RegisterReal(maximum_wind_gust_frequency, "maximum_wind_gust_frequency", 0.0);
    RegisterInteger(wind_gust_frequency_discrete_unit_count, "wind_gust_frequency_discrete_unit_count", 0);

    // Wind profile / log-law parameters
    RegisterReal(wind_evaluation_height, "wind_evaluation_height", 0.0);
    RegisterReal(U10,                    "U10",                    0.0);
    RegisterReal(kappa_parameter,        "kappa_parameter",        0.0);

    // Boolean flags
    RegisterBoolean(clip_speed_nonnegative, "clip_speed_nonnegative", true);
    RegisterBoolean(manual_mean_wind_speed, "manual_mean_wind_speed", true);
    RegisterBoolean(fail_outputs_zero,      "fail_outputs_zero",      true);

    // Inputs (recommended for RL-driven mid-sim changes)
    RegisterInput(mean_wind_speed,         "mean_wind_speed",         0.0);
    RegisterInput(mean_wind_direction_deg, "mean_wind_direction_deg", 0.0);

    // Outputs
    RegisterOutput(wind_speed,         "wind_speed",         0.0);
    RegisterOutput(wind_direction_rad, "wind_direction_rad", 0.0);
    RegisterOutput(wind_direction_deg, "wind_direction_deg", 0.0);
    RegisterOutput(wind_valid,         "wind_valid",         true);

    // Internal state (hidden, but saved with the FMU state; the random generator and gust angles are not saved)
    RegisterInternal(spectrum_is_computed, "spectrum_is_computed", false);
    RegisterInternal(pre_compute,          "pre_compute",          false);
    RegisterInternal(Ubar,                 "Ubar",                 0.0);
    RegisterInternal(dir,                  "dir",                  0.0);
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

// Uniform random number in [0, 1)
static double random_uniform(std::mt19937_64& rng)
{
    return (rng() >> 11) * (1.0 / 9007199254740992.0);
}

// Normal random number N(0,1) (Box-Muller). Written out, because std::normal_distribution
// gives different numbers on different compilers.
static double random_normal(std::mt19937_64& rng)
{
    double u1 = random_uniform(rng);
    double u2 = random_uniform(rng);
    return std::sqrt(-2.0 * std::log(1.0 - u1)) * std::cos(2.0 * std::numbers::pi * u2);
}

// NORSOK Spectrum
double MODEL_IDENTIFIER::_norsok(double f)
{
    // S(f) = 320 (U10/10)^2 (z/10)^0.45 / (1 + x^n)^(5/(3n))
    // x = 172 f (z/10)^(2/3) (U10/10)^(-3/4), n = 0.468
    if (U10 == 0.0)
    {
        // Python raises ZeroDivisionError for 0.0 to a negative power
        throw std::domain_error("0.0 cannot be raised to a negative power");
    }
    double n = 0.468;
    double x = 172.0 * f * std::pow(z / 10.0, 2.0 / 3.0) * std::pow(U10 / 10.0, -3.0 / 4.0);
    return 320.0 * std::pow(U10 / 10.0, 2) * std::pow(z / 10.0, 0.45) / std::pow(1.0 + std::pow(x, n), 5.0 / (3.0 * n));
}

void MODEL_IDENTIFIER::pre_compute_initial_parameters()
{
    // Random seed
    rng.seed(static_cast<std::uint64_t>(seed));  // private RNG

    // Map user params to internal names safely
    z     = wind_evaluation_height;
    kappa = kappa_parameter;

    // Frequency grid (same as np.linspace)
    f_min = minimum_wind_gust_frequency;
    f_max = maximum_wind_gust_frequency;
    N_f   = wind_gust_frequency_discrete_unit_count;
    f.clear();
    if (N_f >= 2 && f_max > f_min)
    {
        double step = (f_max - f_min) / (N_f - 1);
        for (int i = 0; i < N_f; i++)
        {
            f.push_back(i * step + f_min);  // Hz
        }
        f[N_f - 1] = f_max;
        df = f[1] - f[0];
    }

    // Get the initial mean wind speed, available with two mode:
    // 1. Mean wind at height z
    // 2. Manual mean wind at initialization
    if (manual_mean_wind_speed)
    {
        Ubar = initial_mean_wind_speed;
    }
    else
    {
        // Guard kappa and z
        double k    = std::max(1e-9, kappa);
        double z_   = std::max(1e-6, z);
        double z0   = 10.0 * std::exp(-2.0 / (5.0 * std::sqrt(k)));
        Ubar        = U10 * (2.5 * std::sqrt(k)) * std::log(z_ / z0);
    }
    mu_Ubar    = mean_wind_speed_decay_rate;
    sigma_Ubar = mean_wind_speed_standard_deviation;

    // Get the initial wind direction
    dir       = wrap_pi(deg2rad(initial_wind_direction_deg));
    mu_dir    = wind_direction_deg_decay_rate;
    sigma_dir = deg2rad(wind_direction_deg_standard_deviation);

    // Wind speed bound
    Ubar_min = minimum_mean_wind_speed;
    Ubar_max = maximum_mean_wind_speed;

    // Precompute gust amplitudes & phases (fixed), keep running angles
    if ((!spectrum_is_computed) && (!f.empty()))
    {
        a.resize(N_f);
        theta.resize(N_f);
        for (int i = 0; i < N_f; i++)
        {
            double S = _norsok(f[i]);
            a[i]     = std::sqrt(2.0 * S * df);
        }
        for (int i = 0; i < N_f; i++)
        {
            theta[i] = 2 * std::numbers::pi * random_uniform(rng);  // fixed phases, then running angles
        }
        spectrum_is_computed = true;
    }
}

double MODEL_IDENTIFIER::compute_wind_gust(double step_size)
{
    // advance phases by 2 pi f dt and sum components
    double sum = 0.0;
    for (int i = 0; i < N_f; i++)
    {
        theta[i] += 2 * std::numbers::pi * f[i] * step_size;
        sum += a[i] * std::cos(theta[i]);
    }
    return sum;
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

    double a_  = std::exp(-mu * dt);
    double var = (1.0 - std::exp(-2.0 * mu * dt)) / (2.0 * mu);
    return m + (x - m) * a_ + sigma * std::sqrt(var) * random_normal(rng);
}

// =========================
// Main stepping
// =========================
// vel_mean = action outputed by RL agent.
// [vel_mean > 0], negative value will automatically reversed to positive
//
// dir_mean = action outputed by RL agent.
//
// Mean-driven OU:
//   dUbar = -mu_Ubar (Ubar - mean_speed) dt + sigma_Ubar dW
//   dpsi  = -mu_dir wrap(psi - mean_dir) dt + sigma_dir dW
void MODEL_IDENTIFIER::do_step(double current_time, double step_size)
{
    try
    {
        // Pre-compute the initial parameters
        if (!pre_compute)
        {
            pre_compute_initial_parameters();
            pre_compute = true;
        }

        double dt = step_size;
        if (dt <= 0.0)
        {
            // Keep outputs/state unchanged; still "valid"
            wind_valid = true;
            return;
        }

        // Read inputs (master can keep them constant for many steps and change anytime)
        double mean_speed = mean_wind_speed;
        double mean_dir   = deg2rad(mean_wind_direction_deg);

        // Compute wind speed
        // Ubar
        mean_speed = std::abs(mean_speed);  // Nonnegative value
        Ubar       = ou_exact_step(Ubar, mean_speed, mu_Ubar, sigma_Ubar, dt);
        Ubar       = std::min(std::max(Ubar, Ubar_min), Ubar_max);  // np.clip

        // Wind gust
        double Ug = 0.0;
        if (spectrum_is_computed && (!f.empty()) && (U10 > 0.0) && (z > 0.0))
        {
            Ug = compute_wind_gust(dt);
        }

        // Total wind speed
        double wind_speed_new = Ubar + Ug;
        if (clip_speed_nonnegative)
        {
            wind_speed_new = std::max(0.0, wind_speed_new);
        }

        // Compute wind direction
        // Compute direction deviation
        mean_dir   = wrap_pi(mean_dir);
        double err = wrap_pi(dir - mean_dir);

        // Integrate the OU-step for the error
        double err_new = ou_exact_step(err, 0.0, mu_dir, sigma_dir, dt);  // Error OU-step on error mean 0

        // Get the overall wind direction
        dir = wrap_pi(mean_dir + err_new);

        // Assign the results to the output variables
        wind_speed         = wind_speed_new;
        wind_direction_rad = dir;
        wind_direction_deg = rad2deg(dir);
        wind_valid         = true;
    }
    catch (const std::exception& e)
    {
        std::cout << "[WindModel] Exception t=" << current_time << ", dt=" << step_size << ": " << e.what() << std::endl;

        wind_valid = false;
        if (fail_outputs_zero)
        {
            wind_speed         = 0.0;
            wind_direction_rad = 0.0;
            wind_direction_deg = 0.0;
        }
        // else: hold last outputs (do nothing)
    }
}

void MODEL_IDENTIFIER::DoStep(const double globalTime)
{
    do_step(time, globalTime - time);
    SystemBaseWithSolvers::DoStep(globalTime);
}
