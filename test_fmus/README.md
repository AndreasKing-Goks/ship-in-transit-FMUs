# Script for Individual FMU test
The `test_fmus` directory contains **standalone testing scripts for individual FMUs or partially connected FMU subsystems**.

These scripts are intended for:
-   **Unit testing** individual FMUs
-   **Subsystem testing** where several FMUs are connected together
-   **Debugging FMU behavior** without running the full Ship-in-Transit
    simulator
-   **Understanding the functionality and interfaces of each FMU**

Instead of running the full co-simulation framework, these scripts allow developers to **isolate specific FMUs and validate their behavior in a controlled environment**. This makes it easier to:
-   verify input/output variables
-   debug integration issues
-   understand how each subsystem works internally

It is **strongly recommended to review these scripts** when trying tounderstand how each FMU operates.

------------------------------------------------------------------------

## Available Test Scripts

| Script Name                                  | Description                                                                                              |
|----------------------------------------------|----------------------------------------------------------------------------------------------------------|
| `test_autopilot.py`                          | Tests the autopilot FMU responsible for heading control and navigation commands.                         |
| `test_machinery_system.py`                   | Tests the machinery system by altering the throttle and the operating mode.                              |
| `test_machinery_system_raw.py`               | Test of the machinery system FMU with raw interfacing of `libcosimpy`. `CosimInstance` is not used here. |
| `test_mission_manager.py`                    | Tests the mission manager FMU responsible for high-level navigation logic and mission execution.         |
| `test_rudder.py`                             | Tests the rudder FMU responsible for steering dynamics and rudder actuation.                             |
| `test_shaft_speed_controller.py`             | Tests the shaft speed controller FMU that regulates propulsion shaft speed.                              |
| `test_ship-throttle-machinery_system-ship.py`| Integration test connecting ship dynamics, throttle control, and machinery system FMUs.                  |
| `test_ship_model.py`                         | Tests the ship dynamics FMU that simulates vessel motion.                                                |
| `test_ship_path_following.py`                | Tests the path-following ship tracking given waypoints.                                                  |
| `test_surface_current_model.py`              | Tests the environmental model that generates surface current disturbances.                               |
| `test_throttle_controller.py`                | Tests the throttle controller FMU responsible for propulsion command generation.                         |
| `test_throttle-machinery_system.py`          | Integration test connecting throttle control and machinery system FMUs.                                  |
| `test_wind_model.py`                         | Tests the wind disturbance model used in the environmental simulation.                                   |
