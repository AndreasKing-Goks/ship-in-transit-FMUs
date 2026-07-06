# Core Component: `CoSimInstance`
`CoSimInstance` is the **core interface for managing FMU-based co-simulation**. It wraps functionality from the `libcosimpy` library and simplifies the process of setting up communication between FMUs.

The underlying library was developed by **Stian Skjong** (**stian.skjong@sintef.no**) and provides the following core classes:

-   `CosimExecution`
-   `CosimSlave`
-   `CosimManipulator`
-   `CosimObserver`
-   `CosimEnums`

The `CoSimInstance` class abstracts much of the low-level complexity and allows FMUs to be connected and managed with simple method calls.

## FMU Communication Overview
Each FMU exposes variables through **ports**, and these are identified using:

-   **SlaveName** --- the FMU instance name
-   **VarName** --- the variable (port) name

However, the co-simulation engine does **not** use these names internally. Instead, it identifies variables using:
-   **slave index**
-   **variable index**

Therefore, during initialization, the framework resolves the names into the indices required by the co-simulation engine.

### Variable Types
Each variable also has a defined type:
-   `int`
-   `real` (float)
-   `bool`
-   `string`

For two FMUs to communicate, the connected ports must:
1.  Have **matching variable types**
2.  Be mapped through their corresponding **indices**

Once connected, the **output of one FMU** can override the **input of another FMU**.

## Global Simulation Step
The co-simulation advances using a **global step** handled by:

    CosimExecution.step()
This step call **simultaneously advances all FMUs** in the simulation.

## Frequently Used Methods

### `AddSlave()`
Adds an FMU to the simulation by providing the path to the `.fmu` file.

### `AddSlaveConnection()`

Connects the output variable of one FMU to the input variable of another FMU. This establishes the communication link between components.

### `AddObserverTimeSeriesWithLabel()`
Registers a variable for time-series tracking. Parameters include:
-   `slaveName`
-   `variableName`
-   `label`

### `GetObserverTimeSeries()`
Retrieves the recorded time-series of an observed variable.

### `GetLastValue()`
Returns the most recent value of a tracked variable.

### `CoSimManipulate()` *(legacy)*
A simple manipulation method that forces FMUs to communicate with their connected ports **without masking or additional logic**. (*will be explained further below*)

### `SingleVariableManipulation()`
Allows targeted manipulation of a specific FMU variable by injecting a custom input value.

### `AddInputFromExternal()`
Registers FMU variables that will receive input from **external sources** (not from another FMU).

Examples:
-   Reinforcement learning actions
-   External controllers
-   Scenario injection

### `SetInputFromExternal()`
Injects external input values into the FMU variables previously
registered using `AddInputFromExternal()`.

### `SetInitialValue()`
Initializes FMU parameters before the simulation starts. This method sets values for variables declared as **parameters**.

## FMPy support
We also have a separate script for an `FMPy`-based `CoSimInstance` that covers the same functionality with the `libcosimpy`-based `CoSimInstance`, but with additional function which is the `Reset` method and `Terminate` method.

### `Reset()`
Reset every FMU, clear recorded series, and re-apply initial values.

### `Terminate()`
Terminate FMUs and clean up the extracted directories.

------------------------------------------------------------------------

# Orchestrator: `ShipInTransitCoSimulation`

`ShipInTransitCoSimulation` extends the `CoSimInstance` class and is specifically designed for running the **FMU-based Ship-in-Transit Simulator**.

The orchestrator is responsible for:
-   simulator initialization
-   FMU coordination
-   inter-component communication
-   visualization (plots and animations)

Below are the main method used to run and simulate Ship in Transit Cosimulation:

## `AddMap()`
Loads map layers from a **GeoPackage** and stores them as `GeoDataFrame` objects.

Additionally:
-   the land layer is converted to a **Shapely polygon**
-   this enables geometric operations such as **grounding detection**

## `GetShipSpawn()`
Converts ship spawn requests into the dictionary for the ship configuration dictionary compliant with `AddAllShips()` method.

## `AddShipSpawn()`
Adds converted spawn requests to each ship configuration dictionary. This allows ships to be dynamically inserted into the simulation.

## `AddAllShips()`
Add all ships based on its configuration and auto solve FMU connections between each ship entity. Also help adjusts for the Collision Avoidance FMU connection with the other relevant FMUs.

This includes connections required for:
-   **Collision Avoidance FMUs**
-   inter-ship communication

## `CoSimManipulate()`
Handles runtime manipulation between FMUs. This method also supports the **Delayed Start** feature.

### Delayed Start
Delayed start allows a **target ship** to begin simulation **at a time later than `t = 0`**. Because the co-simulation engine only recognizes FMUs (not ships), an additional protocol is required.

Each ship's FMUs follow a naming convention:

    OS0__[FMU_NAME] -> Own Ship
    TS1__[FMU_NAME] -> Target Ship 1
    TS2__[FMU_NAME] -> Target Ship 2
    ...

If a ship has a delayed start time, the inputs of all its FMUs are **masked** until the delayed start time is reached. This ensures the FMUs:
-   remain stable
-   do not evolve dynamically
-   keep the ship stationary

The masking logic is implemented in:

    get_masked_input_val_for_delayed_ship()

## `PreSolverFunctionCall()`
Executed **before each simulation step**. Handles:
-   external input injection (e.g., RL actions)
-   additional FMU manipulation

## `PostSolverFunctionCall()`
Executed **after each simulation step**. Handles:
-   simulation termination checks
-   reward calculation for reinforcement learning

## `step()`
Advances the simulation by one global step.

## `Simulate()`
Runs the simulation loop by repeatedly calling `step()` until a **termination condition** is triggered.

## `JointPlotTimeSeries()`
Plots one or multiple time-series variables tracked by observers. Features:
-   plot individual signals
-   join multiple signals into one plot

Note: Joined signals should ideally share the **same physical unit**.

## `PlotFleetTrajectory()`
Generates static plots of ship trajectories. Supported features:
-   optional background map
-   route overlays
-   waypoint markers
-   Radius-of-Acceptance (RoA) circles
-   ship labels
-   ship hull outlines

### Modes
1.  **Without map**
2.  **With map (`self.is_map_exists == True`)**

## `AnimateFleetTrajectory()`
Creates an animated visualization of ship motion. Supported features:
-   background map
-   route overlays
-   waypoint markers
-   RoA circles
-   ship labels
-   ship hull outlines

### Modes
1.  **Without map**
2.  **With map (`self.is_map_exists == True`)**

## FMPy support
We also have a separate script for an `ShipInTransitCoSimulation`-based `CoSimInstance` that covers the same functionality with the `libcosimpy`-based `ShipInTransitCoSimulation`, but with additional function which is the `Reset` method and `RespawnAndReset` method.

### `Reset()`
Reset the underlying co-simulation (FMUs, observer time series, `self.time` via `CoSimInstance.Reset()`) AND the SIT-level bookkeeping added on top of it, so that repeated `Simulate()` calls on the same instance behave like fresh runs.

### `RespawnAndReset()`

Stage a new scenario's spawn positions and routes on the current co-simulation instance, then immediately apply them by calling `Reset()`. This combines the usual two-step workflow:

1. Stage new spawn data.
2. Call `Reset()`.

into a single convenience function.

This method is intended for efficiently sweeping through many scenarios (e.g., a Monte Carlo bank of `trafficgen` cases) while **reusing the same FMPy co-simulation instance**, thereby avoiding the overhead of repeatedly constructing a new `ShipInTransitCoSimulation`.

The following constraints apply:

- The ship topology **must remain unchanged** across calls. The existing FMU slaves created by `AddAllShips()` are reused.
- Only each ship's:
  - spawn position,
  - route, and
  - `start_time`
  
  may change between scenarios.
- This method **does not** add or remove FMU slaves. Instead, it re-stages the spawn data for the existing ships before performing `Reset()`.

This staging step is required because a ship's initial position, yaw, and speed are FMI 2.0 parameters with **`fixed` variability**. Such parameters can only be modified while the FMU is in **initialization mode**.

Calling `SetInitialValues()` merely stages the values in `self.initial_values`; it does **not** write them to the FMUs. During `Reset()`, the internal `_initialize()` routine re-enters initialization mode, applies the staged values to each FMU, and rebuilds the Ship-in-Transit bookkeeping (e.g., `stop_info`, `ship_reach_end_waypoint`, and related state) using the updated `ship_configs`.
