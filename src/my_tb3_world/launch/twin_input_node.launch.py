from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_tb3_world',
            executable='twin_input_node',
            name='twin_input_node',
            output='screen',
            parameters=[{
                'cell_size_m': 1.0,
                'tick_rate_hz': 1.0,
                'zone_width_m': 7.0,
                'zone_height_m': 7.0,
            }]
        )
    ])
