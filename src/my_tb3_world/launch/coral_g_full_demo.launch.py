#!/usr/bin/env python3
"""
Full CORAL-G demo launcher.

Starts:
  - Gazebo world and TurtleBot3 spawn
  - Nav2 with pre-built map (AMCL localisation) + mission_planner_node
  - CORAL-G digital mission nodes
  - optional RViz

Simulation (default arena_map.yaml):
  ros2 launch my_tb3_world coral_g_full_demo.launch.py use_rviz:=true min_density_reward:=0.001

Lab (physical robot with lab map):
  ros2 launch my_tb3_world coral_g_full_demo.launch.py use_rviz:=true min_density_reward:=0.001 \
    map:=/home/<lab_username>/CORAL-G/src/my_tb3_world/maps/arena_map_lab.yaml

Dashboard is intentionally launched separately from tools/debris_dashboard_web.py
so it can be run from the source checkout on the host that needs browser access.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    my_pkg_share = get_package_share_directory('my_tb3_world')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    initial_x = LaunchConfiguration('initial_x', default='0.0')
    initial_y = LaunchConfiguration('initial_y', default='0.0')
    initial_yaw = LaunchConfiguration('initial_yaw', default='0.0')
    min_density_reward = LaunchConfiguration('min_density_reward', default='0.001')
    map_file = LaunchConfiguration('map', default=os.path.join(
        my_pkg_share, 'maps', 'arena_map.yaml'
    ))

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_pkg_share, 'launch', 'new_world.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'x_pose': initial_x,
            'y_pose': initial_y,
        }.items(),
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_pkg_share, 'launch', 'nav2_localization.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'map': map_file,
            'initial_x': initial_x,
            'initial_y': initial_y,
            'initial_yaw': initial_yaw,
        }.items(),
    )

    coral_g_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_pkg_share, 'launch', 'coral_g_nodes.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'min_density_reward': min_density_reward,
        }.items(),
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock.',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically activate Nav2 and SLAM lifecycle nodes.',
        ),
        DeclareLaunchArgument(
            'initial_x',
            default_value='0.0',
            description='Initial TurtleBot/map-frame x.',
        ),
        DeclareLaunchArgument(
            'initial_y',
            default_value='0.0',
            description='Initial TurtleBot/map-frame y.',
        ),
        DeclareLaunchArgument(
            'initial_yaw',
            default_value='0.0',
            description='Initial yaw estimate in radians.',
        ),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(my_pkg_share, 'maps', 'arena_map.yaml'),
            description=(
                'Full path to saved map yaml. '
                'Defaults to arena_map.yaml for simulation, '
                'pass arena_map_lab.yaml for physical robot.'
            ),
        ),
        DeclareLaunchArgument(
            'min_density_reward',
            default_value='0.001',
            description='Demo planner threshold for normalized density cells.',
        ),
        world_launch,
        TimerAction(period=5.0, actions=[nav2_launch]),
        TimerAction(period=45.0, actions=[coral_g_launch]),
    ])

