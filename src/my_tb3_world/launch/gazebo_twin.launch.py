#!/usr/bin/env python3
"""
CORAL-G Gazebo Twin Launch 

Starts Gazebo with the same CORAL-G arena world used in the physical lab
(new_world.world), then spawns the simulated TurtleBot3 inside the /sim/
namespace so its topics never collide with the physical robot's topics.

Why this matters:
  The physical robot is localised with AMCL against arena_map_lab.yaml
  (the saved map of new_world.world).  Running Gazebo with the same world
  means both physical and simulated robots share identical geometry, so
  the digital twin is spatially consistent with the real environment.

Topic isolation:
  Physical robot  /scan  /odom  /cmd_vel        (from robot SBC over network)
  Gazebo twin     /sim/scan  /sim/odom  /sim/cmd_vel  (from this launch)

twin_safety_node (started by coral_g_twin_demo.launch.py):
  reads  /scan + /sim/scan          obstacle check on both environments
  writes /cmd_vel + /sim/cmd_vel    same command to both robots

Nav2 / AMCL note:
  Only the physical robot runs Nav2 + AMCL (nav2_localization.launch.py,
  use_sim_time:=false, map:=arena_map_lab.yaml).  The Gazebo twin does NOT
  run its own Nav2; it simply mirrors every /cmd_vel_raw command it receives.
  The pre-built map is therefore used exactly once, by the physical robot.

Prerequisites:
  export TURTLEBOT3_MODEL=burger   (must be set before launching)
  Gazebo must NOT already be running (this launch starts its own gz_sim).

TF note:
  robot_state_publisher for the sim robot runs inside PushRosNamespace('sim'),
  so it publishes /sim/robot_description.  The physical robot's TF
  (base_footprint, base_link, etc.) comes from the robot SBC over the network.
  Both publish to the same TF frame names; in practice AMCL's continuous
  /map→/odom update keeps navigation stable because it is authoritative.
  If RViz shows a flickering robot model this is cosmetic only.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    my_pkg_share     = get_package_share_directory('my_tb3_world')
    tb3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')

    # ── Same world file used by the physical lab map ──────────────────────────
    world = os.path.join(my_pkg_share, 'worlds', 'new_world.world')

    # Make TurtleBot3 mesh/model files discoverable by Gazebo
    set_env_vars = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(tb3_gazebo_share, 'models'),
    )

    # Gazebo physics server — headless, loads the CORAL-G arena
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r -s -v2 {world}',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # Gazebo GUI client
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-g -v2',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # ── Sim robot in /sim/ namespace ──────────────────────────────────────────
    # GroupAction + PushRosNamespace prefixes every topic from the included
    # launches with /sim/:
    #   robot_state_publisher  →  /sim/robot_description
    #   ros_gz_bridge (scan)   →  /sim/scan
    #   ros_gz_bridge (cmd)    →  /sim/cmd_vel
    #   ros_gz_bridge (odom)   →  /sim/odom
    sim_robot_group = GroupAction([
        PushRosNamespace('sim'),

        # Publishes /sim/robot_description (needed by spawn_turtlebot3)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_gazebo_share, 'launch', 'robot_state_publisher.launch.py')
            ),
            launch_arguments={'use_sim_time': 'false'}.items(),
        ),

        # Spawns the sim robot into Gazebo and sets up ROS-GZ bridges.
        # With PushRosNamespace active, the spawn topic resolves to
        # /sim/robot_description and all bridge topics become /sim/*.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_gazebo_share, 'launch', 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={
                'x_pose': '0.0',
                'y_pose': '0.0',
            }.items(),
        ),
    ])

    return LaunchDescription([
        set_env_vars,
        gzserver_cmd,
        gzclient_cmd,
        sim_robot_group,
    ])