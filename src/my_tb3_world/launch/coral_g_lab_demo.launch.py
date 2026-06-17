#!/usr/bin/env python3
"""
CORAL-G Lab Demo — CORAL-G nodes + safety stop
For use with the physical TurtleBot3 only.

Map hook:
  nav2_localization.launch.py publishes /map from arena_map_lab.yaml.
  environment_generator_node and digital_twin_state_node subscribe to /map, so
  the lab map remains the source of truth for DT cells and blocked areas.

Gazebo note:
  This launch does not start or mirror Gazebo.  For Option A with the custom
  Gazebo world visible, run gazebo_twin.launch.py and coral_g_twin_demo.launch.py
  instead.

Terminal structure for lab:
  Terminal 1: (nothing — no Gazebo)
  Terminal 2: ros2 launch my_tb3_world nav2_localization.launch.py \\
                use_sim_time:=false
  Terminal 3: ros2 launch my_tb3_world coral_g_lab_demo.launch.py
  Terminal 4: ros2 run my_tb3_world field_planner_node --ros-args \\
                -p use_sim_time:=false \\
                -p min_density_reward:=0.001 \\
                -p goal_wall_clearance_cells:=1 \\
                -p map_cell_size_m:=0.5 \\
                -p plan_rate_hz:=2.0 \\
                -p republish_interval_sec:=2.0
  Terminal 5: rviz2

Prerequisites:
  - ROS_DOMAIN_ID matching between laptop and robot
  - Robot placed at map origin before launching Terminal 2
  - nav2_params_lab.yaml: collision_monitor.cmd_vel_out_topic must be "cmd_vel_raw"
    (safety_stop_node inserted between collision_monitor and the robot)
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

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
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
            default_value='false',
            description='Use simulation clock (false for lab).',
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
                'storage_capacity_items': 8,
                'publish_rate_hz': 0.5,
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
                'tick_rate_hz': 0.5,
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
                'publish_rate_hz': 0.5,
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
                'publish_rate_hz': 0.5,
                'simulation_horizon_sec': 60.0,
                'random_seed': 23,
                'prediction_drift_scale': prediction_drift_scale,
            }],
        ),

        Node(
            package='my_tb3_world',
            executable='debris_viz_node',
            name='debris_viz_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'publish_force_vectors': True,
                'publish_density': True,
            }],
        ),

        Node(
            package='tb3_safety_stop',
            executable='safety_stop_node',
            name='safety_stop_node',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'stop_distance': 0.25,
                'front_angle_deg': 12.0,
                'scan_topic': '/scan',
                'input_cmd_topic': '/cmd_vel_raw',
                'output_cmd_topic': '/cmd_vel',
            }],
        ),

    ])
