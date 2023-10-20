# JS_Optimizer
This project implements different Simulated Annealing Optimizers for the
Job Shop Scheduling problem.

It offers multiple representations for Job Shop problems that are suited for
different iteration methods. The existing representations can potentially be used for
other optimization approaches (Particle Swarm for example).

The project structure is designed to be extended with new representations and new
optimizers.

Currently there is no GUI to configure the optimization, all the configuration is done
by hand inside the main.cpp file in the JS_Optimizer project. The current code should
provide ample usage examples.

Please consult the [documentation](docs/JS_Lib_docs.html) (docs/JS_Lib_docs.html)
to understand the project structure and the currently implemented optimizers.

File formats for problems are covered in the docs.

## Building the project
Building was tested with CMake 3.26 and Visual Studio 2022 on both Windows 10
and Windows 11.

Clone the repository and run CMake from the command line or with the GUI.
(Beginners: first configure, then generate).
This should generate the Visual Studio solution in the selected folder, the solution
can then build the executable.

### Dependencies/Libraries
The project uses Python Matplotlib to visualize the generated solutions.
This functionality is used by running Python scripts manually and thus Python is not
strictly required.

#### Loguru
The logging library Loguru is used, see [emilk/loguru](https://github.com/emilk/loguru).
It is provided in this repository and automatically built.
