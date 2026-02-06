# ROS2-Week-2-Creating-and-building-Packages
This repo is all about creating and building ROS2 packages
🎯 Learning Objectives
By the end of this week, you will be able to:

✅ Understand ROS 2 workspace structure

✅ Create and build ROS 2 packages (Python & C++)

✅ Master package.xml and CMakeLists.txt configuration

✅ Use colcon build system effectively

✅ Create launch files for multi-node systems

✅ Understand dependencies and package relationships

Key Directories:

src/: Your source code - where you develop packages

build/: Intermediate build files (object files, CMake cache)

install/: Final installation - where executables and libraries go

log/: Build logs, warnings, and errors

2.2 Package Types in ROS 2

1. Python Packages (ament_python)

Pure Python packages

Use setup.py for installation

Lighter weight, easier for beginners

2. C++ Packages (ament_cmake)

C/C++ packages

Use CMakeLists.txt for building

Better performance for compute-intensive tasks

3. Hybrid Packages

Can contain both Python and C++ code

More complex configuration

2.3 The Build System: Colcon

Colcon (Collective Construction)

Successor to catkin_make from ROS 1

More flexible and modern

Supports parallel building

Common colcon commands:

# Build all packages in workspace
colcon build

# Build specific package
colcon build --packages-select my_package

# Build with symlink installation (faster development)
colcon build --symlink-install

# Build in release mode
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Build with debugging
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
