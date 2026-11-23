# C++ FMUs (fmicpp)

C++ versions of some of the PythonFMU models in `FMU_scripts/`, written with
SINTEF's fmicpp framework. They have the same variable names, types and
start values as the Python FMUs, so they can replace them by changing the `.fmu` path in a YAML config.

| C++ model                               | Python original                                                | Built FMU in `FMUs_cpp/` |
|-----------------------------------------|----------------------------------------------------------------|--------------------------|
| `simplified_ship/Autopilot`             | `FMU_scripts/simplified_ship/autopilot_fmu.py`                 | `simplified_ship/`       |
| `simplified_ship/Rudder`                | `FMU_scripts/simplified_ship/rudder_model_fmu.py`              | `simplified_ship/`       |
| `simplified_ship/ShipModel`             | `FMU_scripts/simplified_ship/ship_model_fmu.py`                | `simplified_ship/`       |
| `simplified_ship/SpeedController`       | `FMU_scripts/simplified_ship/speed_controller_fmu.py`          | `simplified_ship/`       |
| `mission_manager/MissionManager`        | `FMU_scripts/mission_manager/mission_manager_fmu.py`           | `simplified_ship/`       |
| `mission_manager/MissionManagerAST`     | `FMU_scripts/mission_manager/mission_manager_ast_fmu.py`       | `simplified_ship/`       |
| `ship/ThrottleController`               | `FMU_scripts/ship/throttle_controller_fmu.py`                  | `ship/`                  |
| `ship/ShaftSpeedController`             | `FMU_scripts/ship/shaft_speed_controller_fmu.py`               | `ship/`                  |
| `ship/MachinerySystem`                  | `FMU_scripts/ship/machinery_system_fmu.py`                     | `ship/`                  |
| `ship/SimpleCollisionAvoidance`         | `FMU_scripts/ship/simple_collision_avoidance_fmu.py`           | `ship/`                  |
| `ship/SimpleCollisionAvoidanceReversed` | `FMU_scripts/ship/simple_collision_avoidance_fmu_reversed.py`  | `ship/`                  |
| `env_loads/WindModel`                   | `FMU_scripts/env_loads/wind_model_fmu.py`                      | `env_loads/`             |
| `env_loads/SurfaceCurrentModel`         | `FMU_scripts/env_loads/surface_current_model_fmu.py`           | `env_loads/`             |

The Python Autopilot, Rudder, ShipModel and MissionManager in `FMUs/ship/` are built from the same
scripts as the ones in `FMUs/simplified_ship/`, so the own ship can use the C++ FMUs in
`FMUs_cpp/simplified_ship/` for them.

The FMUs are FMI 2.0 Co-Simulation, Windows (win64) only for now.

The C++ models copy the Python behaviour exactly, also where it may not be intended:
`MachinerySystem` gives the fuel consumption per step, not a running total, and
`ShaftSpeedController` uses `a + floor` as feedforward (the `a * speed + floor` line is commented out).
If the ship reaches the end waypoint before the intermediate waypoint sampler starts, `MissionManagerAST`
gives an error every step from then on and sets `reach_wp_end` back to false.

`MissionManagerAST` keeps its waypoint lists in `std::vector`s. They are not saved with the FMU state.
Its `messages` output is the same text as in Python, with one exception: if `scope_length` is never set,
Python prints the default as `2500` and C++ as `2500.0`.

`WindModel` and `SurfaceCurrentModel` use other random numbers than numpy (`std::mt19937_64`, with a
written-out normal distribution so that Windows and Linux give the same numbers). With noise or wind
gusts on, a C++ run gives other values than a Python run with the same seed, but with the same
statistics. With noise and gusts off they match exactly. The random generator and the gust angles are
not saved with the FMU state.

## How the models work

Each model keeps the Python `do_step()` and runs it once per communication step from an override of
fmicpp's `DoStep()`. The fmicpp solver is turned off (`solverType = "None"`).

Do not put step logic in `SolverPreCalc()` or `SolverPostCalc()`. fmicpp also calls those while the
importer reads outputs during initialization, with a step size of 1.0, and then undoes the changes.

Values that must survive between steps (integrators, previous errors, flags) are registered with
`RegisterInternal()`, so they are saved with the FMU state.

## Known fmicpp 0.5 issue: tunable parameters

fmicpp 0.5 turns `ScalarVariable::Type::TunableParameter` into a fixed parameter. The models set
`variability[...] = Tunable` again after registering, so these parameters can still be changed after
initialization, like in the Python FMUs. But `modelDescription.xml` still says `variability="fixed"`
for them, because the XML writer has the same bug. Remove the extra lines when fmicpp is fixed.

## Build (Windows)

You need:

- Visual Studio 2022 Build Tools (C++ compiler and CMake 3.23 or newer).
- Conan 2. It is in the repo's `environment.yml`.
- Access to the fmicpp Conan packages. `fmicppcore/0.5@fmicpp/stable` must be in your Conan cache,
  or in a Conan remote you are logged in to (the fmicpp GitLab package registry).

The first time, create a default Conan profile:

```powershell
conan profile detect
```

Then build:

```powershell
.\FMU_cpp\build_win.ps1
```
