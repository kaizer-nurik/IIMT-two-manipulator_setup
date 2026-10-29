cmake_minimum_required(VERSION 3.16)
project(iimt_ros2_driver)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)
find_package(rclcpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(iimt_ros2_driver src/iimt_ros2_driver.cpp src/joint_reader.cpp src/main.cpp)

target_include_directories(iimt_ros2_driver
  PRIVATE
  include
)

set_target_properties(iimt_ros2_driver PROPERTIES INSTALL_RPATH "$ORIGIN")

# Vendor library has no SONAME: link by name so installed binaries remain relocatable.
add_library(iimt_vendor_sdk SHARED IMPORTED)
set_target_properties(iimt_vendor_sdk PROPERTIES
  IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/lib/lib64/libiimtcl.so"
  IMPORTED_NO_SONAME TRUE)
target_link_libraries(iimt_ros2_driver iimt_vendor_sdk Threads::Threads)

ament_target_dependencies(iimt_ros2_driver
  rclcpp
  rclcpp_action
  control_msgs trajectory_msgs diagnostic_msgs rcl_interfaces sensor_msgs)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES ERROR_DIAGNOSTICS_RU.md DESTINATION share/${PROJECT_NAME})
install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/lib64/
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
iimt_ros2_driver
  DESTINATION lib/${PROJECT_NAME})

# Export the unchanged vendor ABI declarations for the ros2_control adapter.
# The adapter loads the original shared library dynamically; it does not copy SDK assets.
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
ament_export_include_directories(include/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  # This executable resolves every SDK symbol against an in-process fake. Never
  # link the vendor library: integration tests must not contact hardware.
  add_executable(iimt_driver_error_fixture
    src/iimt_ros2_driver.cpp src/joint_reader.cpp src/main.cpp test/error_sdk_fixture.cpp)
  target_include_directories(iimt_driver_error_fixture PRIVATE include)
  target_link_libraries(iimt_driver_error_fixture Threads::Threads)
  ament_target_dependencies(iimt_driver_error_fixture
    rclcpp rclcpp_action control_msgs trajectory_msgs diagnostic_msgs rcl_interfaces sensor_msgs)
  add_test(NAME iimt_driver_errors_test
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/driver_errors_test.py
      $<TARGET_FILE:iimt_driver_error_fixture>)
  set_tests_properties(iimt_driver_errors_test PROPERTIES TIMEOUT 120 RUN_SERIAL TRUE)
  add_executable(iimt_joint_reader_test test/joint_reader_test.cpp src/joint_reader.cpp)
  target_include_directories(iimt_joint_reader_test PRIVATE include)
  target_link_libraries(iimt_joint_reader_test Threads::Threads)
  add_test(NAME iimt_joint_reader_test COMMAND iimt_joint_reader_test)
  add_executable(iimt_trajectory_test test/trajectory_test.cpp)
  target_include_directories(iimt_trajectory_test PRIVATE include)
  add_test(NAME iimt_trajectory_test COMMAND iimt_trajectory_test)
endif()

ament_package()
