#!/usr/bin/env python3
"""
CORAL-G Localization Launch — SLAM Toolbox with pre-loaded map
Loads a pre-built map into SLAM Toolbox (localization mode) — no driving needed.
Use this for all runs after the map has been saved once with nav2_slam_navigation.launch.py.

Usage:
  ros2 launch my_tb3_world nav2_localization.launch.py
  ros2 launch my_tb3_world nav2_localization.launch.py map:=/home/ubuntu/map/coral_g_map.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    slam_toolbox_share = get_package_share_directory('slam_toolbox')
    my_pkg_share = get_package_share_directory('my_tb3_world')

    # Map saved inside the Docker container by map_saver_cli
    default_map = '/home/ubuntu/map/coral_g_map.yaml'
    coral_g_params = os.path.join(my_pkg_share, 'params', 'coral_g_nav_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    map_yaml = LaunchConfiguration('map', default=default_map)
    params_file = LaunchConfiguration('params_file', default=coral_g_params)
    goal_timeout_sec = LaunchConfiguration('goal_timeout_sec', default='120.0')
    initial_x = LaunchConfiguration('initial_x', default='0.0')
    initial_y = LaunchConfiguration('initial_y', default='0.0')
    initial_yaw = LaunchConfiguration('initial_yaw', default='0.0')

    # SLAM Toolbox in localization mode — loads the saved map, no new mapping
    slam_toolbox_localization = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            coral_g_params,
            {
                'use_sim_time': True,
                'map_file_name': default_map,
                'map_start_at_dock': True,
            }
        ],
    )

    # Map server — serves the saved map to Nav2 costmap
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'yaml_filename': default_map,
        }],
    )

    # Map server lifecycle manager
    map_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    # Nav2 navigation stack — delayed 5 seconds
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
        }.items(),
    )

    # Mission planner — delayed 20 seconds
    mission_planner_node = Node(
        package='my_tb3_world',
        executable='mission_planner_node',
        name='mission_planner_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'goal_timeout_sec': goal_timeout_sec,
            'initial_x': initial_x,
            'initial_y': initial_y,
            'initial_yaw': initial_yaw,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Full path to the saved map yaml file.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=coral_g_params,
            description='Nav2 parameter file.',
        ),
        DeclareLaunchArgument('goal_timeout_sec', default_value='120.0'),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),

        # Map server + SLAM Toolbox localization start immediately
        map_server,
        map_lifecycle,
        slam_toolbox_localization,

        # Nav2 starts after 5 seconds — gives map server time to serve the map
        TimerAction(period=5.0, actions=[nav2]),

        # Mission planner starts after 20 seconds — gives Nav2 time to fully activate
        TimerAction(period=20.0, actions=[mission_planner_node]),
    ])