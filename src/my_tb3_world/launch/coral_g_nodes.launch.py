#!/usr/bin/env python3
"""
CORAL-G DTAS Node Launch File
Launches all digital twin layer nodes:
  - base_reference_node     — latches /base_pose from params (default: origin)
  - robot_state_node        — tracks pose, fuel, storage from /odom + /collection_event
  - environment_generator_node — provides map-aware current/wind/wave field service
  - environment_node        — owns current physical debris truth, publishes env grid,
                              detects waste collections, and emits /dashboard debug data
  - digital_twin_state_node — merges all inputs into /twin_state
  - debris_prediction_node  — publishes normalized debris density cells
  - field_planner_node      — plans from density cells; publishes /next_cell_goal

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
            executable='environment_generator_node',
            name='environment_generator_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'cell_size_m': 0.5,
                'current_strength': 0.3,
                'current_heading_deg': 45.0,
                'wind_x': 0.1,
                'wind_y': 0.05,
                'wave_height': 0.2,
                'confidence': 0.9,
            }],
        ),

        Node(
            package='my_tb3_world',
            executable='environment_node',
            name='environment_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'cell_size_m': 0.5,
                'tick_rate_hz': 1.0,
                'debris_drift_enabled': True,
                'debris_drift_scale': 0.015,
                'physical_debris_seed': 23,
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
                'cell_size_m': 0.5,
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
                'prediction_debris_count': 100,
                'simulation_horizon_sec': 60.0,
                'random_seed': 23,
                'prediction_drift_enabled': True,
                'prediction_drift_scale': 0.015,
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
                'fuel_return_threshold': 0.25,
                'storage_return_threshold': 0.8,
                'density_reward_weight': 1.0,
                'travel_cost_weight': 0.2,
                'storage_penalty_weight': 0.5,
                'fuel_penalty_weight': 0.5,
                'map_risk_weight': 0.5,
                'return_reserve': 0.2,
                'min_density_reward': 0.1,
            }],
        ),

    ])
