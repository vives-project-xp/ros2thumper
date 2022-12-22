#include "rclcpp/rclcpp.hpp"
#include "led_command_cpp/led_command_cpp_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LedCommandCppNode>());
  rclcpp::shutdown();
  return 0;
}