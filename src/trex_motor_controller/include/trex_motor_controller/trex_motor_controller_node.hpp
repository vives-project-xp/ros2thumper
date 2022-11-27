#pragma once

#include "rclcpp/rclcpp.hpp"
#include "trex_motor_controller.hpp"
// #include "std_msgs/msg/string.hpp"
#include "trex_interfaces/msg/drive.hpp"      // generated
#include "trex_interfaces/msg/status.hpp"      // generated
#include "motor_direction.hpp"
#include <stdexcept>

class TRexMotorControllerNode : public rclcpp::Node {
  public:
    TRexMotorControllerNode() : Node("trex_motor_controller") {
      RCLCPP_INFO(this->get_logger(), "Starting up TRex Motor Controller Node ...");

      if (motorController.open()) {
        RCLCPP_INFO(this->get_logger(), "Successfully initialized trex motor controller");
        timer = this->create_wall_timer(
          std::chrono::milliseconds(1000),
          std::bind(&TRexMotorControllerNode::read_status, this)
        );

        statusPublisher = this->create_publisher<trex_interfaces::msg::Status>("trex/status", 10);
            // 10 = required queue size to limit messages in the event of a backup. 

        driveSubscription = this->create_subscription<trex_interfaces::msg::Drive>(
          "trex/drive",         // topic
          10,                   // The depth of the subscription's incoming message queue.
          std::bind(&TRexMotorControllerNode::drive_callback, this, std::placeholders::_1)   // callback takes 1 arg
        );
      }
      else RCLCPP_ERROR(this->get_logger(), "Failed to initialized trex motor controller");
    }

  private:
    void read_status() {
      try {
        TRex::Status status = motorController.status();
        auto message = trex_interfaces::msg::Status();
        message.battery_voltage = status.batteryVoltage;
        
        message.left_motor_speed = status.leftMotorSpeed;
        message.left_motor_direction = (uint8_t)(status.leftMotorDirection);
        message.left_motor_current = status.leftMotorCurrent;
        message.left_motor_braking = status.leftMotorBraking;
        
        message.right_motor_speed = status.rightMotorSpeed;
        message.right_motor_direction = (uint8_t)(status.rightMotorDirection);
        message.right_motor_current = status.rightMotorCurrent;
        message.right_motor_braking = status.rightMotorBraking;

        statusPublisher->publish(message);
        // RCLCPP_INFO(this->get_logger(), "Motor battery voltage: " + std::to_string(status.batteryVoltage));
      } catch (const std::runtime_error& err) {
        RCLCPP_WARN(this->get_logger(), "%s", err.what());
      }
    }

    void drive_callback(trex_interfaces::msg::Drive::UniquePtr msg) {
      RCLCPP_INFO(this->get_logger(), "Received Drive command");
      RCLCPP_INFO(this->get_logger(), "Drive Received: R %d %s - L %d %s",
        msg->left_motor_speed,
        (msg->left_motor_direction == 0 ? "(forward)" : "(backwards)"),
        msg->right_motor_speed,
        (msg->right_motor_direction == 0 ? "(forward)" : "(backwards)")
      );

      try {
        motorController.drive(
          msg->left_motor_speed,
          (TRex::MotorDirection)(msg->left_motor_direction),
          msg->right_motor_speed,
          (TRex::MotorDirection)(msg->right_motor_direction)
        );
      } catch (const std::runtime_error& err) {
        RCLCPP_WARN(this->get_logger(), "%s", err.what());
      }
    }

    rclcpp::TimerBase::SharedPtr timer;
    TRex::TRexMotorController motorController;
    rclcpp::Subscription<trex_interfaces::msg::Drive>::SharedPtr driveSubscription;
    rclcpp::Publisher<trex_interfaces::msg::Status>::SharedPtr statusPublisher;
};