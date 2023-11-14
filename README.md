# JS_Optimization
## Overview
This project implements different Optimizers for the
[Job Shop Scheduling problem](https://en.wikipedia.org/wiki/Job-shop_scheduling).

It offers multiple representations for Job Shop problems that are suited for
different iteration methods. The existing representations can potentially be used for
other optimization approaches (Particle Swarm for example).

The project structure is designed to be extended with new representations and new
optimizers.

Currently there is no GUI to configure the optimization, all the configuration is done
by hand inside the main.cpp file in the JS_Optimizer project. The current code should
provide ample usage examples.

Please consult the documentation to understand the project structure and the currently
implemented optimizers.

File formats for Problem Instances are also covered in the docs.
## Documentation
Documentation can be found at [docs/js_optimization/index.html](docs/js_optimization/index.html).

If anything non-trivial has been omitted please let me know by opening an issue with the
'documentation' label.

## Building the project
Building was tested with CMake 3.26 and Visual Studio 2022 on both Windows 10
and Windows 11.

Clone the repository and run CMake from the command line or with the GUI.
(Beginners: first configure and select the Visual Studio compiler, then generate).
This should generate the Visual Studio solution in the selected build folder, the solution
can then build the executable.

### Dependencies
CMake and Visual Studio (a version that has C++20 support)

#### Python
The project uses Python Matplotlib to visualize the generated solutions.
This functionality is used by running Python scripts manually and thus Python is not
strictly required.

A wrapper Python script for [Google OR Tools](https://developers.google.com/optimization)
was created and can be used to solve Problem instances. It requires some additional setup.
Follow the [Installation Guide](https://developers.google.com/optimization/install/python)
and see the documentation or the -help on the script for usage information.

#### Loguru
The logging library Loguru is used, see [emilk/loguru](https://github.com/emilk/loguru).
Version 2.1.0 is included and built in the JS_Optimzer.sln.

The source code is unaltered.

See his [official documentation](https://emilk.github.io/loguru/index.html) or use a local copy at 'JS_Optimization/docs/loguru/index.html'.

## License
MIT License

Copyright (c) 2023 Alexander Gertsch

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.