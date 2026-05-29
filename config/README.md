## Configuration File

The simulation is configured using a **YAML configuration file (`.yaml`)** that defines the simulation parameters and the ships involved in the scenario. An example structure is shown below:

    config.yaml
    ├── simulation {dict}
    │   ├── instanceName: str
    │   │
    │   ├── stopTime: float
    │   │
    │   ├── stepSize: float
    │   │
    │   └── map {dict}                              # OPTIONAL
    │       ├── name: str
    │       ├── group: str
    │       ├── filename: str
    │       ├── show_coast: bool
    │       ├── show_water: bool
    │       ├── show_waterways: bool
    │       ├── show_ferry_routes: bool
    │       ├── show_harbour: bool
    │       ├── show_bridges: bool
    │       ├── show_tss: bool
    │       └── show_docks: bool
    │
    └── ships {list}
        ├── ship_1 {dict}
        │   ├── id: str
        │   │
        │   ├── sogMax: float                       # OPTIONAL
        │   │
        │   ├── mmsi: int                           # OPTIONAL
        │   │
        │   ├── shipType: str                       # OPTIONAL
        │   │
        │   ├── route_filename: str                 # ATTENTION*
        │   │        
        │   ├── route {dict}                        # ATTENTION*
        │   │   ├── north: list
        │   │   ├── east: list
        │   │   └── speed: list                     # ATTENTION**
        │   │
        │   ├── spawn {dict}                        
        │   │   ├── start_time: float               
        │   │   ├── north: float
        │   │   ├── east: float
        │   │   ├── yaw_angle_deg: float
        │   │   └── forward_speed: float
        │   │
        │   ├── IW_sampling {dict}                  # OPTIONAL
        │   │   ├── active: bool
        │   │   └── animated: bool
        │   │
        │   ├── SHIP_BLOCKS {list}
        │   │   ├── [block_name, path]
        │   │   └── ...
        │   │
        │   ├── SHIP_CONNECTIONS {list}
        │   │   ├── [input_block_name, input_var_name, output_block_name, output_var_name]
        │   │   └── ...
        │   │
        │   ├── SHIP_OBSERVERS {list}
        │   │   ├── [block_name, var_name, var_label]
        │   │   └── ...
        │   │
        │   └── fmu_params {dict}
        │       ├── block_name: block_param {dict}
        │       └── ...
        │
        ├── ship_2 {dict}
        │   └── ...
        │
        └── ...

------------------------------------------------------------------------
## Overview
The simulation is configured using a YAML file (`config.yaml`). This configuration is divided into two main sections:

1. `simulation` (dict)
2. `ships` (list)

## 1. Simulation Configuration

The `simulation` block defines global parameters required to run the co-simulation.

### Required Fields
- **instanceName (str)**  
  Unique identifier for the simulation instance.

- **stopTime (float)**  
  Total simulation duration in seconds.

- **stepSize (float)**  
  Time step for the simulation.

### Optional: Map Configuration

The `map` block is optional and is used for visualization and route integration.

- **name (str)**  
  Title displayed in plots/animations.

- **group (str)**  
  Used to match route directories. Routes are expected under:  
  `data/map/route/<group>/`

- **filename (str)**  
  Name of the GeoPackage (.gpkg) file containing map data.

- Visualization flags:
  - `show_coast`
  - `show_water`
  - `show_waterways`
  - `show_ferry_routes`
  - `show_harbour`
  - `show_bridges`
  - `show_tss`
  - `show_docks`

These flags control which map layers are rendered.

### Optional: Ship Traffic Generator

These fields are optional and needed only when `Ship Traffic Generator` feature is enabled. But even if these values are not specified, the values will be automatically assigned to default values.

- **sogMax (float)**  
  Maximum ship speed over ground in m/s. *Default*: 13 m/s

- **mmsi (int)**  
  Maritime Mobile Service Identity (MMSI). *Default*: 100000001

- **shipType (str)**  
  General ship type based on Automatic Identification System (AIS).
  
  Available type:
  - `Anti-pollution equipment`
  - `Cargo` (*Default*)
  - `Diving operations`
  - `Dredging or underwater operations`
  - `Fishing`
  - `High speed craft`
  - `Law enforcement`
  - `Medical transport`
  - `Military operation`
  - `Noncombatant`
  - `Passenger`
  - `Pilot vessel`
  - `Pleasure craft`
  - `Port tender`
  - `Sailing`
  - `Search and rescue vessel`
  - `Tanker`
  - `Towing`
  - `Towing large`
  - `Tug`
  - `Wing in ground`
  - `Other`
  - `Not available`


## 2. Ships Configuration

Each ship must define a minimal set of parameters to be simulated.

### Required Fields

#### a. id (str)
Unique identifier of the ship. The first ship should only be named as `OS0` to enable orchestrator to recognize the ship as the own ship.

#### b. Spawn Configuration

Defines initial conditions of the ship. Particularly `forward_speed` is needed to set the initial forward speed of the ship when the simulation first begin. Filled `start_time` fields with values bigger than zero to activate the **Delayed Start** feature.

Fields:
- `start_time` (float)
- `north` (float, optional)
- `east` (float, optional)
- `yaw_angle_deg` (float, optional)
- `forward_speed` (float)

#### c. Route Definition

Each ship must define **either**:
- `route`, OR
- `route_filename`

**route**
Manual definition:
- `north`: list
- `east`: list
- `speed`: list.

**route_filename**
- Used when `map` is active.
- The system will look for the `north` and `east` lists of coordinates of the waypoint in:
  `data/map/route/<group>/`
- However the `speed` list still needs to be manually defined.
- `ShipInTransitCosimulation` will then handle this route file and build a structure `route` structure. 

`speed` dictates the desired speed of the ship until the ship visit the next waypoint. That means the `speed` list length is always one element shorter than `north` and `east` list.

#### d. Intermediate Waypoint (IW) Sampling (Optional)

Used for Adaptive Stress Testing (AST).

Fields:
- `active` (bool)
- `animated` (bool)

**active = True**
→ Enables intermediate waypoint sampling.

**animated = True**
→ Displays sampling artifacts in animation (debug/analysis).
→ Can make visualization cluttered.

**Recommendation:**
- Use `animated = False` for clean visualization.
- Use `animated = True` for debugging and research insights.

#### e. SHIP_BLOCKS
Defines FMU components.
Example:
```yaml
SHIP_BLOCKS:
  - [BLOCK_NAME, path/to/file.fmu]
```

#### f. SHIP_CONNECTIONS
Defines signal wiring between blocks.

Example:
```yaml
SHIP_CONNECTIONS:
  - [input_block, input_var, output_block, output_var]
```

#### g. SHIP_OBSERVERS
Defines logged variables.

Example:
```yaml
SHIP_OBSERVERS:
  - [block, variable, label]
```

#### h. fmu_params
Defines parameters for each block.

Example:
```yaml
fmu_params:
  BLOCK_NAME:
    param: value
```

### Important Concept

Each ship is defined by:
- Id → unique identifier
- Route → trajectory ships need to follow
- Spawn → where, when and how each ship is spawn
- Blocks → components
- Connections → system structure
- Parameters → behavior
- Observers → measurements

All four are **REQUIRED** for the simulation to run.


### Minimal Example

```yaml
ships:
  - id: "OS0"

    route:
      north: [0, 1000]
      east: [0, 0]
      speed: [5, 5]

    SHIP_BLOCKS:
      - [SHIP_MODEL, fmus/ship_model.fmu]

    SHIP_CONNECTIONS: 
      - [input_block_name, input_var_name, output_block_name, output_var_name]

    SHIP_OBSERVERS:
      - [SHIP_MODEL, north, "north"]
      - [SHIP_MODEL, east, "east"]

    fmu_params:
      SHIP_MODEL:
        mass: 5000
```

### Adding Multiple Ships

Additional ships can be included by **adding new entries under the `ships` list**.

Example:

``` yaml
ships:
  - ship_1
  - ship_2
  - ship_3
```
## Important Notes

### FMU validity

All FMU files referenced in the configuration must exist and be
accessible from the specified paths.

### Consistent block naming

The `block_name` used in the following sections must be **consistent**:

-   `SHIP_BLOCKS`
-   `SHIP_CONNECTIONS`
-   `SHIP_OBSERVERS`
-   `fmu_params`

The simulation parser relies on the **string identifier of the block
name** to correctly:

-   connect FMUs
-   assign parameters
-   extract observations

If a block name is inconsistent between sections, the configuration will
fail during parsing.



## Example (Conceptual)

Example block reference:

``` yaml
SHIP_BLOCKS:
  - ["autopilot", "fmus/autopilot.fmu"]
  - ["ship_dynamics", "fmus/ship_model.fmu"]
```

Then the same `block_name` must be used elsewhere:

``` yaml
SHIP_CONNECTIONS:
  - ["autopilot", "rudder_angle", "ship_dynamics", "rudder_input"]
```

``` yaml
fmu_params:
  autopilot:
    kp: 1.0
    ki: 0.1
```


## Summary
The configuration is flexible and supports both:
- static YAML-based setup
- dynamic runtime control (e.g., spawn requests)

This design allows scalable and modular simulation setups for complex maritime scenarios.
