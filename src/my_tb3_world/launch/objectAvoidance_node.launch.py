from launch import LaunchDescription
from launch_ros.actions import Node
 
 
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_tb3_world',
            executable='objectAvoidance_node',
            name='objectAvoidance_node',
            output='screen',
        )
    ])
