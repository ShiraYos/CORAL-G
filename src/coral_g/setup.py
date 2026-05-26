from setuptools import find_packages, setup

package_name = "coral_g"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/coral_g_demo.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="CORAL-G Team",
    maintainer_email="team@example.com",
    description="CORAL-G ROS 2 digital twin proof-of-concept nodes.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "twin_input_node = coral_g.ros_nodes:twin_input_main",
            "material_belief_node = coral_g.ros_nodes:material_belief_main",
            "digital_twin_state = coral_g.ros_nodes:digital_twin_main",
            "debris_simulation_node = coral_g.ros_nodes:debris_simulation_main",
            "robot_state_node = coral_g.ros_nodes:robot_state_main",
            "field_planner_node = coral_g.ros_nodes:field_planner_main",
            "mission_planner_node = coral_g.ros_nodes:mission_planner_main",
            "goal_navigation_node = coral_g.ros_nodes:goal_navigation_main",
            "collection_report_node = coral_g.ros_nodes:collection_report_main",
            "demo_control_surface = coral_g.ros_nodes:demo_control_main",
            "contract_check = coral_g.contract_check:main",
        ],
    },
)
