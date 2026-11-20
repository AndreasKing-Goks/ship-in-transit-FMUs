# C++ FMUs (fmicpp)

C++ versions of some of the PythonFMU models in `FMU_scripts/`, written with
SINTEF's fmicpp framework. They have the same variable names, types and
start values as the Python FMUs, so they can replace them by changing the `.fmu` path in a YAML config.

| C++ model                         | Python original                                       |
|-----------------------------------|-------------------------------------------------------|
| `simplified_ship/Autopilot`       | `FMU_scripts/simplified_ship/autopilot_fmu.py`        |
| `simplified_ship/Rudder`          | `FMU_scripts/simplified_ship/rudder_model_fmu.py`     |
| `simplified_ship/ShipModel`       | `FMU_scripts/simplified_ship/ship_model_fmu.py`       |
| `simplified_ship/SpeedController` | `FMU_scripts/simplified_ship/speed_controller_fmu.py` |
| `mission_manager/MissionManager`  | `FMU_scripts/mission_manager/mission_manager_fmu.py`  |

The built FMUs are in `FMUs_cpp/simplified_ship/`. They are FMI 2.0 Co-Simulation, Windows (win64) only for now.

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
