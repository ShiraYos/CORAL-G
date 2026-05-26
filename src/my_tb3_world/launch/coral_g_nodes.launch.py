#!/usr/bin/env python3
"""
CORAL-G Combined Launch File
Launches all digital entity nodes together:
- stateSync_node
- twin_input_node

Note: Run this AFTER launching:
  1. new_world.launch.py (Gazebo)
  2. nav2_slam_navigation.launch.py (Nav2 + SLAM)
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # State Synchronisation Node
        # Subscribes: /odom, /coral_g/navigation_status
        # Publishes:  /coral_g/robot_state
        Node(
            package='my_tb3_world',
            executable='stateSync_node',
            name='stateSync_node',
            output='screen',
        ),

        # Twin Input Node
        # Subscribes: /scan, /odom
        # Publishes:  /coral_g/environment_field, /coral_g/map_sync_update
        Node(
            package='my_tb3_world',
            executable='twin_input_node',
            name='twin_input_node',
            output='screen',
            parameters=[{
                'cell_size_m': 1.0,
                'tick_rate_hz': 1.0,
                'zone_width_m': 7.0,
                'zone_height_m': 7.0,
            }]
        ),

    ])