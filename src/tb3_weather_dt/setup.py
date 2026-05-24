from setuptools import setup

package_name = 'tb3_weather_dt'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dt',
    maintainer_email='dt@example.com',
    description='Gazebo-only weather-driven DT supervisor for TurtleBot3',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'weather_adapter = tb3_weather_dt.weather_adapter:main',
            'twin_safety_supervisor = tb3_weather_dt.twin_safety_supervisor:main',
        ],
    },
)