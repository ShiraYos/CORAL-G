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

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    my_pkg_share = get_package_share_directory('my_tb3_world')
    default_demo_params_file = os.path.join(
        my_pkg_share,
        'params',
        'coral_g_demo.yaml',
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    demo_params_file = LaunchConfiguration(
        'demo_params_file',
        default=default_demo_params_file,
    )
    current_strength = LaunchConfiguration('current_strength', default='0.3')
    current_heading_deg = LaunchConfiguration('current_heading_deg', default='45.0')
    wind_x = LaunchConfiguration('wind_x', default='0.1')
    wind_y = LaunchConfiguration('wind_y', default='0.05')
    wave_height = LaunchConfiguration('wave_height', default='0.2')
    debris_drift_scale = LaunchConfiguration('debris_drift_scale', default='0.015')
    prediction_drift_scale = LaunchConfiguration(
        'prediction_drift_scale',
        default='0.015',
    )
    min_density_reward = LaunchConfiguration(
        'min_density_reward',
        default='0.001',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock.',
        ),
        DeclareLaunchArgument(
            'demo_params_file',
            default_value=default_demo_params_file,
            description='CORAL-G demo parameter profile.',
        ),
        DeclareLaunchArgument(
            'current_strength',
            default_value='0.3',
            description='Base current strength for the environment field.',
        ),
        DeclareLaunchArgument(
            'current_heading_deg',
            default_value='45.0',
            description='Base current heading in degrees.',
        ),
        DeclareLaunchArgument(
            'wind_x',
            default_value='0.1',
            description='Environment wind x component.',
        ),
        DeclareLaunchArgument(
            'wind_y',
            default_value='0.05',
            description='Environment wind y component.',
        ),
        DeclareLaunchArgument(
            'wave_height',
            default_value='0.2',
            description='Environment wave height signal.',
        ),
        DeclareLaunchArgument(
            'debris_drift_scale',
            default_value='0.015',
            description='Physical debris drift scale.',
        ),
        DeclareLaunchArgument(
            'prediction_drift_scale',
            default_value='0.015',
            description='Prediction particle drift scale.',
        ),
        DeclareLaunchArgument(
            'min_density_reward',
            default_value='0.001',
            description=(
                'Demo planner threshold for normalized density cells. The node '
                'default remains conservative at 0.1.'
            ),
        ),

        Node(
            package='my_tb3_world',
            executable='base_reference_node',
            name='base_reference_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
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
                'use_sim_time': use_sim_time,
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
            parameters=[demo_params_file, {
                'use_sim_time': use_sim_time,
                'cell_size_m': 0.5,
                'current_strength': current_strength,
                'current_heading_deg': current_heading_deg,
                'wind_x': wind_x,
                'wind_y': wind_y,
                'wave_height': wave_height,
                'confidence': 0.9,
            }],
        ),

        Node(
            package='my_tb3_world',
            executable='environment_node',
            name='environment_node',
            output='screen',
            parameters=[demo_params_file, {
                'use_sim_time': use_sim_time,
                'cell_size_m': 0.5,
                'tick_rate_hz': 1.0,
                'debris_drift_enabled': True,
                'debris_drift_scale': debris_drift_scale,
                'physical_debris_seed': 23,
            }],
        ),

        Node(
            package='my_tb3_world',
            executable='digital_twin_state_node',
            name='digital_twin_state_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'publish_rate_hz': 1.0,
                'cell_size_m': 0.5,
            }],
        ),

        Node(
            package='my_tb3_world',
            executable='debris_prediction_node',
            name='debris_prediction_node',
            output='screen',
            parameters=[demo_params_file, {
                'use_sim_time': use_sim_time,
                'publish_rate_hz': 1.0,
                'simulation_horizon_sec': 60.0,
                'random_seed': 23,
                'prediction_drift_scale': prediction_drift_scale,
            }],
        ),

        # Node(
        #     package='my_tb3_world',
        #     executable='field_planner_node',
        #     name='field_planner_node',
        #     output='screen',
        #     parameters=[demo_params_file, {
        #         'use_sim_time': use_sim_time,
        #         'plan_rate_hz': 0.5,
        #         'fuel_return_threshold': 0.25,
        #         'storage_return_threshold': 0.8,
        #         'density_reward_weight': 1.0,
        #         'travel_cost_weight': 0.2,
        #         'storage_penalty_weight': 0.5,
        #         'fuel_penalty_weight': 0.5,
        #         'map_risk_weight': 0.5,
        #         'return_reserve': 0.2,
        #         'min_density_reward': min_density_reward,
        #     }],
        # ),

    ])
