cmake_minimum_required(VERSION 3.16)
project(iimt_ros2_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

install(
  DIRECTORY config srdf rviz launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
