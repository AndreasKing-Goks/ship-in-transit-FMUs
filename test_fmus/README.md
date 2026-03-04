# Script for Individual FMU test
TBA

<!-- In `test_beds`, we provide several example scripts to better understand how the simulator and environment work:

### `ast_test`
Scripts for testing the AST training scripts.

| Script | Description |
|--------|-------------|
| `run_ast_env.py` | Run the stress-testing simulation environment with the AST action sampling process.  |
| `run_non_ast_env.py` | Run the stress-testing simulation environment without the AST action sampling process.  |
| `setup.py` | A script for setting the configuration of `ship_model` and the environment wrapper class. |
| `test_ast.py` | Main script for running the AST training process. |

### `env_load_model`
Scripts for testing environmental models.  
See [here](https://github.com/AndreasKing-Goks/MAR-AST/tree/main/test_beds/env_load_model).

| Script | Description |
|--------|-------------|
| `current_model_test.py` | Test the current model using `current_model.py` subsystem  |
| `wave_model_test.py` | Test the wave model using `wave_model.py` subsystem |
| `wind_model_test.py` | Test the wind model using `wind_model.py` subsystem |

### `map_and_route_plotter`
Scripts for placing maps and ship routes for visualization — useful for arranging route waypoints before simulation.  
See [here](https://github.com/AndreasKing-Goks/MAR-AST/tree/main/test_beds/map_and_route_plotter).

| Script | Description |
|--------|-------------|
| `plot_map_route.py` | Plot routes stored in `test_beds/map_route_plotter/data` with maps manually designed using `PolygonObstacle()` class. |
| `plot_realmap_route.py` | Plot routes stored in `data/route` with real-world maps retrieved from [Open Street Map (OSM)](https://www.openstreetmap.org/). |

### `ship_simu_test`
Scripts for using the environment classes from `env_wrappers`.  
See [here](https://github.com/AndreasKing-Goks/MAR-AST/tree/main/test_beds).

| Script | Description |
|--------|-------------|
| `test_ship_env_load.py` | Run `MultiShipEnv()` class for single-ship cases without running machinery system to test environment load effects on the ship. |
| `test_single_ship_map.py` | Run `MultiShipEnv()` class on a real-world map while simulating single ship asset. |
| `test_double_ship_map.py` | Run `MultiShipEnv()` class on a real-world map while simulating two ship asset. Used to check the COLAV system. |
| `test_multi_ship_map.py` | Run `MultiShipEnv()` class on a real-world map with multiple ship asset simulated together. | -->