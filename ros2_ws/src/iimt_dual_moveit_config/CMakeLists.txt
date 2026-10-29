cmake_minimum_required(VERSION 3.8)
project(iimt_dual_moveit_config)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch srdf rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/workspace_scene.py DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  enable_testing()
  add_test(NAME iimt_workspace_geometry
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/workspace_test.py)
  add_test(NAME iimt_dual_moveit_config_regression
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/launch_setup_test.py)
endif()
ament_package()
