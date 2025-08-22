# RobotFace-Healthmate-App Map Integration with ros2_qt_control_app

## Overview
This integration allows the RobotFace-Healthmate-App to display real-time maps from ROS2, receive robot position updates, and send navigation goals.

## Features
- ✅ Real-time map visualization from ROS2 `/map` topic
- ✅ Live robot position tracking with orientation
- ✅ Interactive goal setting by clicking on the map
- ✅ Connection status monitoring
- ✅ Map information display (resolution, size)

## Architecture

### Components Added:
1. **Ros2MapBridge** (C++/Qt): Bridge between ROS2 and QML
   - Subscribes to `/map` topic for occupancy grid
   - Publishes to `/goal_pose` for navigation
   - Tracks robot pose via TF2 (map → base_link)

2. **MapScreen.qml**: Enhanced map display with:
   - Dynamic map rendering
   - Robot position marker
   - Goal setting interface
   - Control panel with status info

3. **Launch Files**:
   - `robot_face_app.launch.py`: Standalone app launch
   - `integrated_map_system.launch.py`: Full system with ros2_qt_control_app

## Build Instructions

```bash
# Navigate to workspace
cd /data/hard_disk/nhat_tung/ros2_ws

# Build the package
colcon build --packages-select robot_app

# Source the workspace
source install/setup.bash
```

## Running the System

### Option 1: Full Integrated System
```bash
# This launches both ros2_qt_control_app and RobotFace app
ros2 launch robot_app integrated_map_system.launch.py
```

### Option 2: Standalone RobotFace App
```bash
# Run this if map server is already running
ros2 launch robot_app robot_face_app.launch.py
```

### Option 3: Testing with Simulated Map
```bash
# Terminal 1: Run test map publisher
python3 /data/hard_disk/nhat_tung/ros2_ws/src/test_integration.py

# Terminal 2: Run the RobotFace app
ros2 launch robot_app robot_face_app.launch.py
```

## Usage Instructions

1. **Open Map Screen**: Click "🗺️ Bản đồ" button in the main interface
2. **Connect to ROS2**: Click "Connect to ROS2" button in the map screen
3. **View Map**: The map will automatically display when available
4. **Set Navigation Goal**: Click anywhere on the map to send a goal
5. **Monitor Status**: View robot position and map info in the control panel

## Topics Used

- **Subscribed**:
  - `/map` (nav_msgs/OccupancyGrid): Map data
  - TF2: `map` → `base_link` transform for robot pose

- **Published**:
  - `/goal_pose` (geometry_msgs/PoseStamped): Navigation goals

## Troubleshooting

### No Display Error
If you get "could not connect to display" error:
```bash
# Set display variable (for SSH/remote connections)
export DISPLAY=:0

# Or run with xvfb for headless testing
xvfb-run -a ros2 launch robot_app robot_face_app.launch.py
```

### Map Not Showing
1. Check if map is being published: `ros2 topic echo /map`
2. Verify robot TF is available: `ros2 run tf2_ros tf2_echo map base_link`
3. Ensure you clicked "Connect to ROS2" in the app

### Build Errors
1. Ensure all dependencies are installed:
```bash
sudo apt update
sudo apt install ros-humble-nav-msgs ros-humble-tf2-ros ros-humble-tf2-geometry-msgs
sudo apt install qtbase5-dev qtdeclarative5-dev qtmultimedia5-dev
```

## Files Modified/Added

### New Files:
- `include/ros2mapbridge.h`: ROS2 map bridge header
- `src/ros2mapbridge.cpp`: ROS2 map bridge implementation
- `launch/robot_face_app.launch.py`: Standalone launch file
- `launch/integrated_map_system.launch.py`: Integrated launch file

### Modified Files:
- `CMakeLists.txt`: Added ROS2 dependencies
- `src/main.cpp`: Integrated ROS2 initialization
- `forms/pages/MapScreen.qml`: Dynamic map display
- `forms/pages/RobotFaceScreen.qml`: Pass mapBridge to MapScreen

## Future Enhancements
- Add path visualization when navigation is active
- Display multiple robots on the same map
- Add map editing capabilities
- Integrate with navigation feedback for goal status
