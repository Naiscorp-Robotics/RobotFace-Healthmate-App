#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Launch RobotFace-Healthmate-App node
    robot_face_app_node = Node(
        package='robot_app',
        executable='robot_app',
        name='robot_face_healthmate_app',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        remappings=[
            # Map topics - these should match your actual ROS2 topics
            ('/map', '/map'),
            ('/goal_pose', '/goal_pose'),
        ]
    )
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    
    # Add node
    ld.add_action(robot_face_app_node)
    
    return ld
