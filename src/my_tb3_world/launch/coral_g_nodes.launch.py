#!/usr/bin/env python3
"""
CORAL-G DTAS Node Launch File
Launches all digital twin layer nodes:
  - robot_state_node      — tracks pose, fuel, storage from /odom + /collection_event
  - environment_node      — publishes env grid and detects waste collections
  - digital_twin_state_node — merges all inputs into /twin_state

Run AFTER:
  1. new_world.launch.py           (Gazebo)
  2. nav2_slam_navigation.launch.py (Nav2 + SLAM + mission_planner_node)
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package='my_tb3_world',
            executable='robot_state_node',
            name='robot_state_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'storage_capacity_items': 3,
                'fuel_drain_rate': 0.05,
                'fuel_low_threshold': 20.0,
                'base_radius_m': 0.5,
            }],
        ),

        Node(
            package='my_tb3_world',
            executable='environment_node',
            name='environment_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'cell_size_m': 1.0,
                'tick_rate_hz': 1.0,
                'collection_radius_m': 1.0,
            }],
        ),

        Node(
            package='my_tb3_world',
            executable='digital_twin_state_node',
            name='digital_twin_state_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'publish_rate_hz': 1.0,
            }],
        ),

    ])