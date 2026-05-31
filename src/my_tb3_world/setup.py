from setuptools import setup
import os
from glob import glob

package_name = 'my_tb3_world'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='test',
    maintainer_email='test@todo.todo',
    description='CORAL-G autonomous ocean cleanup system',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_node = my_tb3_world.publisher_node:main',
            'subscriber_node = my_tb3_world.subscriber_node:main',
            'objectAvoidance_node = my_tb3_world.objectAvoidance_node:main',
            # DTAS nodes
            'robot_state_node = my_tb3_world.robot_state_node:main',
            'environment_node = my_tb3_world.environment_node:main',
            'digital_twin_state_node = my_tb3_world.digital_twin_state_node:main',
            'mission_planner_node = my_tb3_world.mission_planner_node:main',
        ],
    },
)