#!/bin/bash

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Create build directory if it doesn't exist
mkdir -p build
cd build

# Configure with CMake
cmake ..

# Build
make

echo "Build complete."
echo "Run with: ./run.sh (recommended to source ROS2 environment)"
echo "Or manually: source /opt/ros/humble/setup.bash && ./build/robot_app"
