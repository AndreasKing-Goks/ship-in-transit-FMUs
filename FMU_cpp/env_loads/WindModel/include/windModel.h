#ifndef WIND_MODEL_H
#define WIND_MODEL_H
#include "modelName.h"
#include "systemBaseWithSolvers.h"

#include <random>
#include <vector>

// Wind Model C++ FMU implementation.
// Mean-driven Ornstein-Uhlenbeck (OU) / Gauss-Markov process for wind, plus wind gusts from a NORSOK spectrum.
// - RL (or external controller) drives the *mean* wind speed and direction.
// - The FMU outputs a stochastic wind that reverts toward those means with inertia.
// Port of FMU_scripts/env_loads/wind_model_fmu.py
//
// The random numbers are not the same as numpy's, so with noise or gusts on the results
// only match the Python FMU statistically.
class MODEL_IDENTIFIER : public SystemBaseWithSolvers
{
public:
    MODEL_IDENTIFIER(std::shared_ptr<LoggerBase> _logger = {}, std::string _location = "");

    // Runs do_step() once per communication step
    void DoStep(const double globalTime) override;

private:
    void do_step(double current_time, double step_size);
    double _norsok(double f);
    void pre_compute_initial_parameters();
    double compute_wind_gust(double step_size);
    double ou_exact_step(double x, double m, double mu, double sigma, double dt);

    // Parameters (fixed)
    int seed;

    double initial_mean_wind_speed;
    double mean_wind_speed_decay_rate;
    double mean_wind_speed_standard_deviation;

    double initial_wind_direction_deg;
    double wind_direction_deg_decay_rate;
    double wind_direction_deg_standard_deviation;

    double minimum_mean_wind_speed;
    double maximum_mean_wind_speed;
    double minimum_wind_gust_frequency;
    double maximum_wind_gust_frequency;
    int wind_gust_frequency_discrete_unit_count;

    double wind_evaluation_height;
    double U10;
    double kappa_parameter;

    bool clip_speed_nonnegative;

    bool manual_mean_wind_speed;

    // What to do on failure
    // If true: outputs become 0 and valid=false
    // If false: hold last outputs and valid=false
    bool fail_outputs_zero;

    // Inputs (time-varying commands from RL/master)
    double mean_wind_speed;
    double mean_wind_direction_deg;

    // Outputs
    double wind_speed;
    double wind_direction_rad;
    double wind_direction_deg;
    bool wind_valid;

    // Internal state
    bool spectrum_is_computed;
    bool pre_compute;
    std::mt19937_64 rng;
    double Ubar;
    double dir;
    double mu_Ubar = 0.0;
    double mu_dir = 0.0;
    double sigma_Ubar = 0.0;
    double sigma_dir = 0.0;
    double z = 0.0;
    double kappa = 0.0;
    double f_min = 0.0;
    double f_max = 0.0;
    int N_f = 0;
    double Ubar_min = 0.0;
    double Ubar_max = 0.0;

    // Gust internals (empty f means no gusts, like f = None in Python)
    std::vector<double> f;
    double df = 0.0;
    std::vector<double> a;
    std::vector<double> theta;
};

#endif
