#!/usr/bin/env python3
"""
Nav2 navigation with a pre-built map (AMCL localisation, no SLAM).
Use this for all runs after the map has been saved once with nav2_slam_navigation.launch.py.

Save the map first (while SLAM is running):
  ros2 run nav2_map_server map_saver_cli -f /ws/src/my_tb3_world/maps/arena_map

Launch order:
  1. new_world.launch.py           — Gazebo
  2. nav2_localization.launch.py   — map server + AMCL + Nav2 + mission planner
  3. coral_g_nodes.launch.py       — DTAS nodes
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
    my_pkg_share       = get_package_share_directory('my_tb3_world')

    default_map_file    = os.path.join(my_pkg_share, 'maps',   'arena_map.yaml')
    default_params_file = os.path.join(my_pkg_share, 'params', 'nav2_params.yaml')

    use_sim_time     = LaunchConfiguration('use_sim_time',     default='true')
    autostart        = LaunchConfiguration('autostart',        default='true')
    map_file         = LaunchConfiguration('map',              default=default_map_file)
    params_file      = LaunchConfiguration('params_file',      default=default_params_file)
    goal_timeout_sec = LaunchConfiguration('goal_timeout_sec', default='120.0')
    initial_x        = LaunchConfiguration('initial_x',        default='0.0')
    initial_y        = LaunchConfiguration('initial_y',        default='0.0')
    initial_yaw      = LaunchConfiguration('initial_yaw',      default='0.0')

    # map_server + AMCL (localization_launch handles the lifecycle manager too)
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart':    autostart,
            'map':          map_file,
            'params_file':  params_file,
        }.items(),
    )

    # Nav2 navigation stack — delayed to let AMCL start first
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart':    autostart,
            'params_file':  params_file,
        }.items(),
    )

    # Mission planner — uses 'amcl' localizer so waitUntilNav2Active waits for amcl_pose
    mission_planner = Node(
        package='my_tb3_world',
        executable='mission_planner_node',
        name='mission_planner_node',
        output='screen',
        parameters=[{
            'use_sim_time':     use_sim_time,
            'goal_timeout_sec': goal_timeout_sec,
            'initial_x':        initial_x,
            'initial_y':        initial_y,
            'initial_yaw':      initial_yaw,
            'localizer':        'amcl',
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',  default_value='true'),
        DeclareLaunchArgument('autostart',     default_value='true'),
        DeclareLaunchArgument('map',           default_value=default_map_file,
                              description='Full path to arena_map.yaml'),
        DeclareLaunchArgument('params_file',   default_value=default_params_file),
        DeclareLaunchArgument('goal_timeout_sec', default_value='120.0'),
        DeclareLaunchArgument('initial_x',     default_value='0.0'),
        DeclareLaunchArgument('initial_y',     default_value='0.0'),
        DeclareLaunchArgument('initial_yaw',   default_value='0.0'),

        localization,
        TimerAction(period=5.0,  actions=[navigation]),
        TimerAction(period=15.0, actions=[mission_planner]),
    ])
