#include "rclcpp/rclcpp.hpp"
#include "thumper_drive/thumper_drive_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThumperDriveNode>());
  rclcpp::shutdown();
  return 0;
}