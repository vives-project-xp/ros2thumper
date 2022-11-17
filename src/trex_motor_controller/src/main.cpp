#include "rclcpp/rclcpp.hpp"
#include "trex_motor_controller/trex_motor_controller_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TRexMotorControllerNode>());
  rclcpp::shutdown();
  return 0;
}