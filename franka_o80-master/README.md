# Welcome to `Franka_o80 1.2.0`!
Here you will find a library for control of [Franka Emika Panda](https://www.franka.de/) robot. The library is a specialization of [o80](https://github.com/intelligent-soft-robots/o80) templates and is based on [libfranka](https://github.com/frankaemika/libfranka). It includes both `C++` headers and `Python` bindings.

### Contents
1. [Welcome to Franka_o80](#welcome-to-franka_o80)
2. [Contents](#contents)
3. [Structure](#structure)
4. [Requirements](#requirements)
5. [Building](#building)
6. [Installation](#installation)
7. [Documentation](#documentation)
8. [Notes](#notes)
9. [Contributors](#contributors)

### Example
Then simplest `C++` example is:
```
#include <franka_o80/front_end.hpp>
int main()
{
	franka_o80::FrontEnd frontend("ID"); //ID is a placeholder for shared memory identifier, it needs to be the same for front- and backend
	frontend.add_command(franka_o80::robot_mode, franka_o80::RobotMode::intelligent_position, o80::Mode::QUEUE);
	frontend.add_command(franka_o80::joint_position[0], 1.0, o80::Duration_us::seconds(5), o80::Mode::QUEUE);
	frontend.pulse_and_wait();
}
```
It can be built with the following `CMakeLists.txt`:
```
project(example)
cmake_minimum_required(VERSION 3.14.0)
find_package(Franka_o80 1.0.0 REQUIRED)
add_executable(example example.cpp)
target_link_libraries(example PRIVATE franka_o80)
```
Also, the program requires a running backend, which can be started with the command:
```
franka_o80_backend ID IP & #ID is same identifier as above, IP is the robot IP address
```
Correspondent `Python` example follows:
```
import o80
import franka_o80
frontend = franka_o80.FrontEnd("ID")
frontend.add_command(franka_o80.robot_mode(), franka_o80.State(franka_o80.RobotMode.intelligent_position), o80.Mode.QUEUE)
frontend.add_command(franka_o80.joint_position(0), franka_o80.State(1.0), o80.Duration_us.seconds(5), o80.Mode.QUEUE)
frontend.pulse_and_wait()
```

### Structure
Some important files and directories:
 - `include` - include directory for `C++` programmers
 - `src` - directory containing `C++` sources
 - `example` - directory containing finished `Franka_o80` projects in form of `C++` sources or `Python` files
 - `build` - default name for `cmake` build directory
 - `build/libfranka_o80.so` - shared library for `C++` programmers (could be linked with `link_libraries(Franka_o80)`)
 - `build/franka_o80.cpython-*-*.so` - shared library for `Python` programmers (could be imported with `import franka_o80`)
 - `build/franka_o80_backend` - executable for backend control
 - `build/franka_o80_selftest` - executable for testing the library with [Google Test](https://github.com/google/googletest)
 - `build/franka_o80_control` - example executable for robot control
 - `build/franka_o80_control.py` - example `Python` script for robot control
 - `build/franka_o80_control_trajectory` - example of commands that can be executed with `franka_o80_control`

### Requirements
`Franka_o80` requires:
 - [o80](https://github.com/intelligent-soft-robots/o80)
 - [libfranka](https://github.com/frankaemika/libfranka) (set with `-DFranka_DIR=/absolute_path_to_libfranka/build` or as part of `ROS`)
 - [pinocchio](https://stack-of-tasks.github.io/pinocchio) (set with `-Dpinocchio_DIR=/absolute_path_to_pinocchio` as part of `ROS`)
 - [Eigen](https://eigen.tuxfamily.org)
 - [Boost](https://www.boost.org) (`system` and `thread`)
 - [Google Test](https://github.com/google/googletest) (optionally)
 - [pybind11](https://github.com/pybind/pybind11) (optionally)
 - [Doxygen](https://www.doxygen.nl/index.html) (optionally)
 - [CMake](https://cmake.org) >= `3.14.0`
 - Fully preemptable Linux kernel
 - C++17 compatible compiler
 - `root` privileges

### Building
`Franka_o80` can be built with [CMake](https://cmake.org) using following commands:
```1.0
mkdir build
cd build
cmake ..
cmake --build .
```

### Installation
```
mkdir build
cd build
cmake ..
cmake --build .
sudo cmake --install .
sudo ldconfig
#Further steps are required only if you plan to use pybind'ded classes from Franka_o80 in your pybind'ded library
cmake .. -DFranka_o80_omit_include_directories=yes
sudo cmake --build .
cmake --install .
```

### Documentation
`Python` docstrings are provided. `C++` code is documented with comments. [Doxygen](https://www.doxygen.nl) documentation may be generated with `doxygen` command. Example projects print human-readable help messages.

### Notes
`Franka_o80` may sometimes be non-intuitive. So here are some important notes:
 - **CAUTION!** To make `Python` bindings work properly, it is required to install [o80](https://github.com/intelligent-soft-robots/o80) from source and add `-Do80_DIR=/absolute_path_to_o80_source/install`. It is an issue of [o80](https://github.com/intelligent-soft-robots/o80) and will be fixed in it's future releases.
 - `Franka_o80` is a specialization of some [o80](https://github.com/intelligent-soft-robots/o80) templates. Some vital general functions may be implemented and documented in [o80](https://github.com/intelligent-soft-robots/o80), not `Franka_o80`.
 - The project consists of frontend and backend. **Frontend** is responsible for sending commands and receiving observations from backend. `FrontEnd` class is the main class to be used by programmers in `C++` or `Python`. **Backend** is responsible for communication with the [libfranka](https://github.com/frankaemika/libfranka) and needs to be started on the machine in order to use frontends. This could be done with `franka_o80_backend` executable or with some functions, like `start_standalone`.
 - All variables in `Franka_o80` are implemented as actuators in terms of [o80](https://github.com/intelligent-soft-robots/o80), even control mode, reset signal, error, velocities, torques, etc., and they are controlled with `FrontEnd::add_command` like real actuators. When reading observations, all actuators are defined. But some of them (like `joint_position` and `cartesian_position`) obviously contradict each other, and which of them will be used to control the robot, is decided by `robot_mode` and `gripper_mode` actuators.
 - All actuators, like `robot_mode` or `joint_position`, are just constant numbers. They are only useful in `FrontEnd::add_command`, `States::get` and `States::set` functions.
 - FrontEnd does not throw exceptions when an error in backend occurs. The only way to know about backend errors is to read `control_error` actuator.
 - All `FrontEnd::add_command` functions accept `State`. This is why the class encapsulates both real numbers, quaternions, mode and error enumerations, and some dynamic typization is done in the class (even in `C++`). Backend will, for example, expect the state applied to `robot_mode` actuator to contain `RobotMode` value, but frontend does not check state types and does not throw exceptions. The only way to know that `Frontend::add_command` was called with wrong state type is to read `control_error` actuator.
 - The gripper is controlled in non-[o80](https://github.com/intelligent-soft-robots/o80) style. The gripper is moved with the velocity specified in `gripper_velocity`, duration given to `FrontEnd::add_command` has no influence on it.

### Contributors
 - Kyrylo Sovailo
 - Original [o80](https://github.com/intelligent-soft-robots/o80) is authored by [Vincent Berenz](http://vincentberenz.is.tuebingen.mpg.de), Max Planck Institute for Intelligent Systems, Empirical Inference Department
 - Models from `model` directory were generated with [MoveIt Setup Assistant](http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html) from `ROS` package `franka_description`
