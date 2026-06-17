#!/usr/bin/env python3
"""
Show the Cartographer start pose inside the saved map.

Run with RViz open to see a disk + arrow at the map-frame pose used by
nav2_localization.launch.py defaults: x=0, y=0, yaw=0.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    frame_id = LaunchConfiguration('frame_id', default='map')
    start_x = LaunchConfiguration('start_x', default='0.0')
    start_y = LaunchConfiguration('start_y', default='0.0')
    start_yaw = LaunchConfiguration('start_yaw', default='0.0')

    return LaunchDescription([
        DeclareLaunchArgument(
            'frame_id',
            default_value='map',
            description='Frame containing the saved map.',
        ),
        DeclareLaunchArgument(
            'start_x',
            default_value='0.0',
            description='Cartographer start x in map frame.',
        ),
        DeclareLaunchArgument(
            'start_y',
            default_value='0.0',
            description='Cartographer start y in map frame.',
        ),
        DeclareLaunchArgument(
            'start_yaw',
            default_value='0.0',
            description='Cartographer start yaw in radians.',
        ),
        Node(
            package='my_tb3_world',
            executable='cartographer_start_marker_node',
            name='cartographer_start_marker_node',
            output='screen',
            parameters=[{
                'frame_id': frame_id,
                'start_x': start_x,
                'start_y': start_y,
                'start_yaw': start_yaw,
            }],
        ),
    ])
