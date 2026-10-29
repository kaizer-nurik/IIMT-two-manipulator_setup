#include "iimt_ros2_driver.h"
#include <exception>
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<iimt_ros2_driver::iimtRos2Driver>();
    if (node->start()) {
      // Один executor для команд и action; чтение выполняет отдельный поток.
      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(node);
      executor.spin();
    } else {
      result = 1;
    }
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("iimt_ros2_driver"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
