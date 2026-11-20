#ifndef SURFACE_CURRENT_MODEL_H
#define SURFACE_CURRENT_MODEL_H
#include "modelName.h"
#include "systemBaseWithSolvers.h"

#include <random>

// Surface Current Model C++ FMU implementation.
// Mean-driven Ornstein-Uhlenbeck (OU) / Gauss-Markov process for surface current.
// - RL (or external controller) drives the *mean* current speed and direction.
// - The FMU outputs a stochastic current that reverts toward those means with inertia.
// Port of FMU_scripts/env_loads/surface_current_model_fmu.py
//
// The random numbers are not the same as numpy's, so with noise on the results
// only match the Python FMU statistically.
class MODEL_IDENTIFIER : public SystemBaseWithSolvers
{
public:
    MODEL_IDENTIFIER(std::shared_ptr<LoggerBase> _logger = {}, std::string _location = "");

    // Runs do_step() once per communication step
    void DoStep(const double globalTime) override;

private:
    void do_step(double current_time, double step_size);
    void pre_compute_initial_parameters();
    double ou_exact_step(double x, double m, double mu, double sigma, double dt);

    // Parameters (fixed)
    int seed;

    double initial_current_speed;
    double current_speed_decay_rate;                  // mu_vel [1/s]
    double current_speed_standard_deviation;          // sigma_vel [units of speed / sqrt(s)] for SDE

    double initial_current_direction_deg;
    double current_direction_deg_decay_rate;          // mu_dir [1/s]
    double current_direction_deg_standard_deviation;  // sigma_dir [rad / sqrt(s)] for SDE

    bool clip_speed_nonnegative;

    // What to do on failure
    // If true: outputs become 0 and valid=false
    // If false: hold last outputs and valid=false
    bool fail_outputs_zero;

    // Inputs (time-varying commands from RL/master)
    double mean_current_speed;
    double mean_current_direction_deg;

    // Outputs
    double current_speed;
    double current_direction_rad;
    double current_direction_deg;
    bool current_valid;

    // Internal state
    bool pre_compute;
    std::mt19937_64 rng;
    double vel;
    double dir;
    double mu_vel = 0.0;
    double mu_dir = 0.0;
    double sigma_vel = 0.0;
    double sigma_dir = 0.0;
};

#endif
