# Building FMUs with `PythonFMU`
For a detailed explanation of how `PythonFMU` works, refer to the official repository:

https://github.com/NTNU-IHB/PythonFMU

This section provides a **quick guide** for building an FMU from a Python script within this project.

------------------------------------------------------------------------

## 1. Create an FMU Script
To generate an FMU using PythonFMU, you must create a Python script that **extends the `Fmi2Slave` class**.

Example structure:

``` python
from pythonfmu import Fmi2Slave

class MyFMU(Fmi2Slave):

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # define inputs, outputs, and parameters
        self.input_signal = 0.0
        self.output_signal = 0.0
        self.parameter = 1.0
```

The FMU class defines the variables that will be exposed to the co-simulation environment.

------------------------------------------------------------------------

## 2. Register FMU Variables
After defining variables as class attributes, they must be **registered with the FMU interface** using:

    self.register_variable()
This step tells the FMU which variables are:
-   **inputs**
-   **outputs**
-   **parameters**

Refer to the example scripts inside:

    FMU_script/
or the official PythonFMU documentation for detailed usage.

------------------------------------------------------------------------

## 3. Implement the `do_step()` Method
Every PythonFMU model must implement the method:

    do_step(current_time, step_size)

This function is called by the co-simulation engine at every simulation step.

Example:

``` python
def do_step(self, current_time, step_size):

    # retrieve input
    u = self.input_signal

    # perform model update
    self.output_signal = u * self.parameter

    return True
```

Typical tasks inside `do_step()` include:
-   retrieving inputs from registered variables
-   computing the system dynamics or control logic
-   storing results in output variables

Any **helper methods** required for the computation can also be implemented within the class.

------------------------------------------------------------------------

## 4. Generate the FMU
Once the FMU script is ready, the FMU can be built using the `pythonfmu` command-line tool.

    pythonfmu build -f path/to/fmu_script.py

This command packages the script and generates a `.fmu` file that can be used in the co-simulation framework.

------------------------------------------------------------------------

## 5. Including External Dependencies
If the FMU relies on external Python packages, they must be listed in a `requirements.txt` file.

In this project, dependency files are typically stored in:

    FMU_script/reqs/

Example `requirements.txt`:

    numpy
    scipy

To build the FMU with dependencies:

    pythonfmu build -f path/to/fmu_script.py path/to/requirements.txt

The dependencies will be bundled inside the generated FMU so they areavailable during simulation.

------------------------------------------------------------------------

## Summary
To build an FMU using PythonFMU:
1.  Create a Python class extending `Fmi2Slave`
2.  Define and register FMU variables (inputs, outputs, parameters)
3.  Implement the `do_step()` simulation method
4.  Build the FMU using `pythonfmu build`
5.  Include external dependencies using `requirements.txt` if needed

The resulting `.fmu` file can then be integrated into the Ship-in-Transit co-simulation framework.