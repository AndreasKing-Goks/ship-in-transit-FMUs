# Ship
## Ship Dynamics

The ship model simulates motion in **three degrees of freedom**:

- **Surge**
- **Sway**
- **Yaw**

### State Variables

The full system state includes:

- `x_N`: North position [m]
- `y_E`: East position [m]
- `ψ`: Yaw angle [rad]
- `u`: Surge velocity [m/s]
- `v`: Sway velocity [m/s]
- `r`: Yaw rate (turn rate) [rad/s]

> ⚠️ **ANGLE IN NED FRAME**
>
> When we first initiate the ship heading, `0°` starts from `NORTH (y+)` direction. **Positive heading** is clockwise in direction.


**Additional states** (depending on machinery system model):

- `ω_prop`: Propeller shaft angular velocity [rad/s] — for detailed machinery system
- `T`: Thrust force [N] — for simplified model

### Forces Modeled

- Inertial forces  
- Added mass effects  
- Coriolis forces  
- Linear and nonlinear damping  
- Environmental forces (wind and current)  
- Control forces (propeller & rudder)

## Machinery System

The ship includes:
- **1 propeller shaft**
- **1 rudder**
- Powered by either a **main diesel engine** or a **hybrid shaft generator**.

### Energy Sources

- **Main Engine (ME)**: Primary mechanical power source  
- **Hybrid Shaft Generator (HSG)**: Can operate as motor/generator  
- **Electrical Distribution**: Powered by diesel generators

### Machinery Modes

| Mode   | Propulsion Power       | Hotel Load Power        |
|--------|------------------------|-------------------------|
| PTO    | Main Engine            | Hybrid SG (generator)   |
| MEC    | Main Engine            | Electrical Distribution |
| PTI    | Hybrid SG (motor)      | Electrical Distribution |

### Available Machinery Models

1. **Detailed Model** — includes propeller shaft dynamics  
2. **Simplified Model** — thrust modeled as a 2nd order transfer function  

## Navigation System

Two navigation modes:

- **Heading-by-Reference**: Maintains a specified heading.  
- **Waypoint Controller**: Follows a sequence of waypoints using Line of Sight (LOS) guidance.

## Environmental Forces

- **Wind** : Gust component based from NORSOK Spectrum. Stochasticity is drawn from Ornstein-Uhlenbeck process.  
- **Current** : Stochasticity is drawn from Ornstein-Uhlenbeck process.  

## Speed Control Options

1. **Throttle Input**: A direct command representing propulsion load percentage  
2. **Shaft Speed Controller**: Regulates propeller shaft speed to maintain desired vessel speed  

##  Example Scenarios

- Single-engine configuration using only the Main Engine in PTO mode  
- Complex hybrid-electric propulsion control  

---
