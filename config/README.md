## Configuration File

The simulation is configured using a **YAML configuration file
(`.yaml`)** that defines the simulation parameters and the ships
involved in the scenario.

An example structure is shown below:

    config.yaml
    ├── simulation {dict}
    │   ├── instanceName: str
    │   │
    │   ├── stopTime: float
    │   │
    │   ├── stepSize: float
    │   │
    │   └── map {dict}
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
        │   ├── spawn {dict}
        │   │   ├── start_time: float
        │   │   ├── north: float
        │   │   ├── east: float
        │   │   ├── yaw_angle_deg: float
        │   │   └── forward_speed: float
        │   │
        │   ├── route {dict}
        │   │   ├── north: list
        │   │   ├── east: list
        │   │   └── speed: list
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

## Adding Multiple Ships

Additional ships can be included by **adding new entries under the
`ships` list**.

Each ship must define its own:

-   spawn\
-   route\
-   FMU blocks\
-   block connections\
-   observers\
-   block parameters

Example:

``` yaml
ships:
  - ship_1
  - ship_2
  - ship_3
```

------------------------------------------------------------------------

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

------------------------------------------------------------------------

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
