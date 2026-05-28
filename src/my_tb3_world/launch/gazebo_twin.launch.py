from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    """
    CORAL-G Gazebo Twin Launch
    Spawns a simulated TurtleBot3 into the already-running CORAL-G Gazebo world.
    Must be launched AFTER new_world.launch.py is running.
    """

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/sim/robot_description',
            '-name', 'turtlebot3_sim',
            '-x', '0',
            '-y', '0',
            '-z', '0.1'
        ],
        output='screen'
    )

    return LaunchDescription([
        PushRosNamespace('sim'),
        spawn_robot
    ])