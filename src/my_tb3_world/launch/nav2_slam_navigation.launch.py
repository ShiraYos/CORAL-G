#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Get package directories
    nav2_bringup_share = get_package_share_directory("nav2_bringup")
    my_pkg_share = get_package_share_directory("my_tb3_world")

    # Use the project's custom params (tuned progress checker + inflation for 4x4m arena)
    default_params_file = os.path.join(my_pkg_share, "params", "nav2_params.yaml")

    # Launch configurations
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    autostart = LaunchConfiguration("autostart", default="true")
    params_file = LaunchConfiguration("params_file", default=default_params_file)
    goal_timeout_sec = LaunchConfiguration("goal_timeout_sec", default="120.0")
    initial_x = LaunchConfiguration("initial_x", default="0.0")
    initial_y = LaunchConfiguration("initial_y", default="0.0")
    initial_yaw = LaunchConfiguration("initial_yaw", default="0.0")

    # SLAM Toolbox — launches first
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, "launch", "slam_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "autostart": autostart,
        }.items(),
    )

    # Nav2 navigation stack — delayed 5 seconds to let SLAM initialize first
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "params_file": params_file,
        }.items(),
    )

    # Mission planner node — delayed 10 seconds to let Nav2 initialize first
    mission_planner_node = Node(
        package="my_tb3_world",
        executable="mission_planner_node",
        name="mission_planner_node",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "goal_timeout_sec": goal_timeout_sec,
            "initial_x": initial_x,
            "initial_y": initial_y,
            "initial_yaw": initial_yaw,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock. Set false on the physical robot.",
        ),
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Automatically activate Nav2 and SLAM lifecycle nodes.",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params_file,
            description="Nav2 parameter file.",
        ),
        DeclareLaunchArgument(
            "goal_timeout_sec",
            default_value="120.0",
            description="Seconds before an active goal is canceled as timed out.",
        ),
        DeclareLaunchArgument(
            "initial_x",
            default_value="0.0",
            description="Initial map-frame x pose estimate.",
        ),
        DeclareLaunchArgument(
            "initial_y",
            default_value="0.0",
            description="Initial map-frame y pose estimate.",
        ),
        DeclareLaunchArgument(
            "initial_yaw",
            default_value="0.0",
            description="Initial yaw estimate in radians.",
        ),
        # SLAM starts immediately
        slam,
        # Nav2 starts after 5 seconds — gives SLAM time to initialize
        TimerAction(period=5.0, actions=[nav2]),
        # Mission planner starts after 10 seconds — gives Nav2 time to be ready
        TimerAction(period=10.0, actions=[mission_planner_node]),
    ])