#!/bin/bash

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Publish a message to the sample_topic
ros2 topic pub /sample_topic std_msgs/msg/String "data: 'Test message from ROS2 publisher'"

