#!/usr/bin/env python3
"""
CORAL-G DTAS Node Launch File
Launches all digital twin layer nodes:
  - base_reference_node     — latches /base_pose from params (default: origin)
  - robot_state_node        — tracks pose, fuel, storage from /odom + /collection_event
  - environment_node        — publishes env grid and detects waste collections
  - digital_twin_state_node — merges all inputs into /twin_state
  - debris_prediction_node  — maps uncollected clusters onto /debris_density_map
  - field_planner_node      — autonomous planner; publishes /next_cell_goal

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
            executable='base_reference_node',
            name='base_reference_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'base_x': 0.0,
                'base_y': 0.0,
                'base_yaw': 0.0,
            }],
        ),

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

        Node(
            package='my_tb3_world',
            executable='debris_prediction_node',
            name='debris_prediction_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'publish_rate_hz': 1.0,
            }],
        ),

        Node(
            package='my_tb3_world',
            executable='field_planner_node',
            name='field_planner_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'plan_rate_hz': 0.5,
                'fuel_return_threshold': 0.15,
                'storage_return_threshold': 1.0,
            }],
        ),

    ])