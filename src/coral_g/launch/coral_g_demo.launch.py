from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    node_names = [
        "twin_input_node",
        "material_belief_node",
        "digital_twin_state",
        "debris_simulation_node",
        "robot_state_node",
        "field_planner_node",
        "mission_planner_node",
        "goal_navigation_node",
        "collection_report_node",
        "demo_control_surface",
    ]
    return LaunchDescription(
        [
            Node(package="coral_g", executable=name, name=name, output="screen")
            for name in node_names
        ]
    )
