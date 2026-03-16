# Environmental Loads FMUs

The Environmental Loads group contains FMUs responsible for generating the external environmental disturbances acting on the ship. These disturbances represent ocean and atmospheric effects that influence vessel motion during the simulation.

The models are designed to produce **time-varying stochastic environmental fields**, allowing the simulator to represent realistic operating conditions rather than deterministic constant disturbances.

---

## Environmental Forces

### Wind
The wind model generates a **time-varying wind field** consisting of:

1. **Mean wind component**
2. **Turbulent gust component**

The **mean wind speed and direction** evolve according to a **stochastic Ornstein–Uhlenbeck (OU) process**, which models a mean-reverting random process:
- Wind conditions fluctuate randomly
- The process gradually relaxes toward a specified mean value
- The fluctuations remain physically bounded and temporally correlated

The **gust component** is generated using a **NORSOK wind spectrum**, which represents the frequency distribution of wind turbulence in offshore environments.

This results in a wind signal that contains:
- Low-frequency mean wind variation
- High-frequency gust disturbances
- Realistic temporal correlation

The wind model outputs:
- Wind speed
- Wind direction (degrees and radians)

These values are later used by the **ship dynamics model** to compute aerodynamic forces and moments acting on the vessel.

---

### Surface Current

The surface current model generates a **time-varying ocean current field** using a stochastic process.

Both the **current speed** and **current direction** are modeled using an **Ornstein–Uhlenbeck process**, which provides:
- Mean-reverting stochastic behavior
- Smooth temporal evolution
- Physically realistic variability

This approach ensures that the current:
- Does not jump unrealistically between values
- Gradually evolves around a specified mean current condition

The current model outputs:
- Current speed
- Current direction (degrees and radians)

These values are fed into the ship dynamics model to compute **hydrodynamic current forces acting on the vessel**.

---

## Summary of Environmental Models

| Environmental Model | Distribution / Process | Purpose |
|---------------------|-----------------------|--------|
| Wind Mean Component | Ornstein–Uhlenbeck process | Generates realistic time-varying wind field |
| Wind Gust Component | NORSOK wind spectrum | Models turbulent gust disturbances |
| Surface Current | Ornstein–Uhlenbeck process | Generates stochastic ocean current variations |

---

## Design Rationale

Environmental disturbances in real maritime operations are **highly stochastic and temporally correlated**.  
Using Ornstein–Uhlenbeck processes allows the simulator to reproduce these properties while remaining computationally efficient for co-simulation and reinforcement learning experiments.

---

# FMUs Description

## Surface Current Model FMU

### Inputs
| Attribute Name | Default Value | Description | Type |
|----------------|--------------|-------------|------|
| `mean_current_speed` |  | Mean current speed used as the drift reference for the stochastic current model. | `REAL` |
| `mean_current_direction_deg` |  | Mean current direction (degrees) used as the drift reference for the stochastic current direction model. | `REAL` |

### Outputs
| Attribute Name | Default Value | Description | Type |
|----------------|--------------|-------------|------|
| `current_speed` |  | Generated ocean current speed magnitude. | `REAL` |
| `current_direction_rad` |  | Generated ocean current direction in radians. | `REAL` |
| `current_direction_deg` |  | Generated ocean current direction in degrees. | `REAL` |
| `current_valid` |  | Boolean flag indicating whether the generated current values are valid. | `BOOLEAN` |

### Parameters
| Attribute Name | Default Value | Description | Type |
|----------------|--------------|-------------|------|
| `seed` | 0 | Random seed used to initialize the stochastic current model. | `INTEGER` |
| `initial_current_speed` | 0.01 | Initial current speed at simulation start. | `REAL` |
| `current_speed_decay_rate` | 0.5 | Decay rate governing the relaxation of the current speed toward its mean value. | `REAL` |
| `current_speed_standard_deviation` | 0.15 | Standard deviation of the stochastic current speed process. | `REAL` |
| `initial_current_direction_deg` | 0.0 | Initial current direction at simulation start (degrees). | `REAL` |
| `current_direction_deg_decay_rate` | 0.5 | Decay rate governing the relaxation of the current direction toward its mean value. | `REAL` |
| `current_direction_deg_standard_deviation` | 5.0 | Standard deviation of the stochastic current direction process. | `REAL` |
| `clip_speed_nonnegative` | True | Boolean flag indicating whether current speed should be clipped to nonnegative values. | `BOOLEAN` |
| `fail_outputs_zero` | True | Determines failure behavior: if true, outputs are set to zero when the model fails; otherwise the last valid values are held. | `BOOLEAN` |

---

## Wind Model FMU

### Inputs
| Attribute Name | Default Value | Description | Type |
|----------------|--------------|-------------|------|
| `mean_wind_speed` |  | Mean wind speed command used as the reference for the stochastic wind model. | `REAL` |
| `mean_wind_direction_deg` |  | Mean wind direction command (degrees) used as the reference for the stochastic wind direction model. | `REAL` |

### Outputs
| Attribute Name | Default Value | Description | Type |
|----------------|--------------|-------------|------|
| `wind_speed` |  | Generated wind speed magnitude. | `REAL` |
| `wind_direction_rad` |  | Generated wind direction in radians. | `REAL` |
| `wind_direction_deg` |  | Generated wind direction in degrees. | `REAL` |
| `wind_valid` |  | Boolean flag indicating whether the generated wind values are valid. | `REAL` |

### Parameters
| Attribute Name | Default Value | Description | Type |
|----------------|--------------|-------------|------|
| `seed` | 0 | Random seed used to initialize the stochastic wind model. | `INTEGER` |
| `initial_mean_wind_speed` | 5.0 | Initial mean wind speed at simulation start. | `REAL` |
| `mean_wind_speed_decay_rate` | 0.5 | Decay rate governing the relaxation of the mean wind speed toward its target value. | `REAL` |
| `mean_wind_speed_standard_deviation` | 1.5 | Standard deviation of the stochastic mean wind speed process. | `REAL` |
| `initial_wind_direction_deg` | 0.0 | Initial wind direction at simulation start (degrees). | `REAL` |
| `wind_direction_deg_decay_rate` | 0.5 | Decay rate governing the relaxation of the wind direction toward its target value. | `REAL` |
| `wind_direction_deg_standard_deviation` | 5.0 | Standard deviation of the stochastic wind direction process. | `REAL` |
| `minimum_mean_wind_speed` | 0.0 | Minimum allowable mean wind speed. | `REAL` |
| `maximum_mean_wind_speed` | 42.0 | Maximum allowable mean wind speed. | `REAL` |
| `minimum_wind_gust_frequency` | 0.06 | Minimum gust frequency used in the wind gust spectrum. | `REAL` |
| `maximum_wind_gust_frequency` | 0.4 | Maximum gust frequency used in the wind gust spectrum. | `REAL` |
| `wind_gust_frequency_discrete_unit_count` | 100 | Number of discrete frequency bins used to represent the gust spectrum. | `INTEGER` |
| `wind_evaluation_height` | 5.0 | Height above the sea surface at which the wind speed is evaluated. | `REAL` |
| `U10` | 10.0 | Reference wind speed at 10 meters above the sea surface. | `REAL` |
| `kappa_parameter` | 0.0026 | Log-law or profile parameter used in wind field evaluation. | `REAL` |
| `clip_speed_nonnegative` | True | Boolean flag indicating whether wind speed should be clipped to nonnegative values. | `BOOLEAN` |
| `manual_mean_wind_speed` | True | Boolean flag indicating whether the mean wind speed is directly commanded externally. | `BOOLEAN` |
| `fail_outputs_zero` | True | Determines failure behavior: if true, outputs are set to zero when the model fails; otherwise the last valid values are held. | `BOOLEAN` |