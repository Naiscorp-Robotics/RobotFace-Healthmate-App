#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Package directories
    ros2_qt_control_app_dir = get_package_share_directory('ros2_qt_control_app')
    robot_app_dir = get_package_share_directory('robot_app')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Include ros2_qt_control_app launch file if it exists
    ros2_qt_control_app_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros2_qt_control_app_dir, 'launch', 'qt_control_app.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
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
            # Map topics if needed
            ('/map', '/map'),
            ('/goal_pose', '/goal_pose'),
        ]
    )
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    
    # Add nodes and includes
    ld.add_action(ros2_qt_control_app_launch)
    ld.add_action(robot_face_app_node)
    
    return ld
