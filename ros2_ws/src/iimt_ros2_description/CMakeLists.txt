cmake_minimum_required(VERSION 3.16)
project(iimt_ros2_description)
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
