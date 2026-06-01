cmake_minimum_required(VERSION 3.8)
project(my_tb3_world_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EnvironmentCell.msg"
  "srv/GenerateEnvironmentField.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
