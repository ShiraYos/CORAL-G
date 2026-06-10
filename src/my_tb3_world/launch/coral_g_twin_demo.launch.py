#!/usr/bin/env python3
"""
CORAL-G Lab Demo — Track A (Physical Robot + Gazebo Digital Twin)

Identical to coral_g_lab_demo.launch.py except safety_stop_node is replaced
by twin_safety_node, which gates forward motion from the real robot's /scan
and publishes the safe command to both /cmd_vel and /sim/cmd_vel.

Terminal structure for Track A lab session:
  Terminal 1 (SSH to robot):
      source /opt/ros/jazzy/setup.bash
      export TURTLEBOT3_MODEL=burger
      export LDS_MODEL=LDS-02
      ros2 launch turtlebot3_bringup robot.launch.py

  Terminal 2 (laptop — Gazebo twin):
      export TURTLEBOT3_MODEL=burger
      ros2 launch my_tb3_world gazebo_twin.launch.py

  Terminal 3 (laptop — Nav2 + AMCL):
      ros2 launch my_tb3_world nav2_localization.launch.py \\
        use_sim_time:=false

  Terminal 4 (laptop — DTAS + twin safety):
      ros2 launch my_tb3_world coral_g_twin_demo.launch.py

  Terminal 5 (laptop — field planner):
      ros2 run my_tb3_world field_planner_node --ros-args \\
        -p use_sim_time:=false \\
        -p min_density_reward:=0.001 \\
        -p goal_wall_clearance_cells:=1 \\
        -p map_cell_size_m:=0.5 \\
        -p plan_rate_hz:=2.0 \\
        -p republish_interval_sec:=2.0

  Terminal 6 (laptop):
      rviz2

Prerequisites:
  - ROS_DOMAIN_ID matching between laptop and robot (set before every terminal)
  - Robot placed at map origin before launching Terminal 3
  - nav2_params_lab.yaml: collision_monitor.cmd_vel_out_topic must be "cmd_vel_raw"
    (twin_safety_node sits between collision_monitor and both robots)

How the connection works:
  Nav2 → velocity_smoother → cmd_vel_smoothed
       → collision_monitor  → /cmd_vel_raw
       → twin_safety_node   → /cmd_vel      (real physical robot moves)
                            → /sim/cmd_vel  (Gazebo twin mirrors movement)

  Obstacle stop:
    Real LIDAR  (/scan) → twin_safety_node → blocks both if obstacle < stop_distance
    Gazebo LIDAR (/sim/scan) is not used for stopping in this lab launch

Demo evidence for checklist:
  Bidirectional pub/sub:
    Physical→Twin : ros2 topic echo /robot_state        (real robot state flowing in)
    Twin→Physical : ros2 topic echo /next_cell_goal     (planner driving real robot)
  State synchronization (non-motion):
    ros2 topic echo /twin_state   (fuel, storage, at_base all synced)
  Environmental interaction:
    Place obstacle in real world → both real robot and Gazebo twin command stream stop
    ros2 topic echo /twin_state to confirm robot state reflects event
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

    use_sim_time        = LaunchConfiguration('use_sim_time',        default='false')
    demo_params_file    = LaunchConfiguration('demo_params_file',    default=default_demo_params_file)
    current_strength    = LaunchConfiguration('current_strength',    default='0.3')
    current_heading_deg = LaunchConfiguration('current_heading_deg', default='45.0')
    wind_x              = LaunchConfiguration('wind_x',              default='0.1')
    wind_y              = LaunchConfiguration('wind_y',              default='0.05')
    wave_height         = LaunchConfiguration('wave_height',         default='0.2')
    debris_drift_scale  = LaunchConfiguration('debris_drift_scale',  default='0.015')
    prediction_drift_scale = LaunchConfiguration('prediction_drift_scale', default='0.015')
    min_density_reward  = LaunchConfiguration('min_density_reward',  default='0.001')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',        default_value='false',
                              description='Use simulation clock (false for lab).'),
        DeclareLaunchArgument('demo_params_file',    default_value=default_demo_params_file,
                              description='CORAL-G demo parameter profile.'),
        DeclareLaunchArgument('current_strength',    default_value='0.3'),
        DeclareLaunchArgument('current_heading_deg', default_value='45.0'),
        DeclareLaunchArgument('wind_x',              default_value='0.1'),
        DeclareLaunchArgument('wind_y',              default_value='0.05'),
        DeclareLaunchArgument('wave_height',         default_value='0.2'),
        DeclareLaunchArgument('debris_drift_scale',  default_value='0.015'),
        DeclareLaunchArgument('prediction_drift_scale', default_value='0.015'),
        DeclareLaunchArgument('min_density_reward',  default_value='0.001',
                              description='Planner threshold for normalised density cells.'),

        # ── DTAS nodes (identical to coral_g_lab_demo) ───────────────────────

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

        # ── Safety layer — CHANGED from coral_g_lab_demo ─────────────────────
        # twin_safety_node replaces safety_stop_node.
        # For the lab run, only the real /scan gates safety.  Gazebo still gets
        # the safe mirrored command on /sim/cmd_vel, but /sim/scan cannot stop
        # the physical robot if the Gazebo world differs from the saved map.
        Node(
            package='tb3_safety_stop',
            executable='twin_safety_node',
            name='twin_safety_node',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'real_scan_topic':  '/scan',
                'sim_scan_topic':   '/sim/scan',
                'use_sim_scan_for_stop': False,
                'input_cmd_topic':  '/cmd_vel_raw',
                'real_cmd_topic':   '/cmd_vel',
                'sim_cmd_topic':    '/sim/cmd_vel',
                'stop_distance':    0.30,
                'front_angle_deg':  30.0,
            }],
        ),
    ])
