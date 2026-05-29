from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_tb3_world',
            executable='digital_twin_state_node',
            name='digital_twin_state_node',
            output='screen',
            parameters=[{
                'publish_rate_hz': 1.0,
            }]
        )
    ])
