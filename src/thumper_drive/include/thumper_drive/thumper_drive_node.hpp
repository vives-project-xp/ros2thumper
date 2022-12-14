#pragma once

#include "rclcpp/rclcpp.hpp"
#include "trex_interfaces/msg/drive.hpp"
#include "trex_interfaces/msg/status.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <cmath>

class ThumperDriveNode : public rclcpp::Node {
  public:
    ThumperDriveNode() : Node("thumper_drive") {
      RCLCPP_INFO(this->get_logger(), "Starting Thumper drive Node ...");

      joySubscriber = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy",          // topic
        10,              // The depth of the subscription's incoming message queue.
        std::bind(&ThumperDriveNode::joy_callback, this, std::placeholders::_1)   // callback takes 1 arg
      );

      trexPublisher = this->create_publisher<trex_interfaces::msg::Drive>("trex/drive", 10);
    }

  private:
    void joy_callback(sensor_msgs::msg::Joy::UniquePtr msg) {
      // TODO: Need to drop some messages. Joy is publishing to fast. Tried topic_tools drop but can't build it.
      // Crude solution for getting this to work
      if ((++counter) % 2 != 0) return;

      RCLCPP_INFO(this->get_logger(), "Processing Joystick Message");

      double forward = msg->axes[1];
      double turning = msg->axes[0];

      auto message = trex_interfaces::msg::Drive();

      double speedMotorRight = forward * maxSpeed + turning * maxTurningSpeed;
      double speedMotorLeft = forward * maxSpeed - turning * maxTurningSpeed;

      if (std::abs(speedMotorLeft) > 10) {
        message.left_motor_speed = (uint8_t)(std::abs(speedMotorLeft));
      }

      if (std::abs(speedMotorRight) > 10) {
        message.right_motor_speed = (uint8_t)(std::abs(speedMotorRight));
      }

      if (speedMotorLeft < 0) {
        message.left_motor_direction = 0;
      } else {
        message.left_motor_direction = 1;
      }

      if (speedMotorRight < 0) {
        message.right_motor_direction = 0;
      } else {
        message.right_motor_direction = 1;
      }
      trexPublisher->publish(message);
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySubscriber;
    rclcpp::Publisher<trex_interfaces::msg::Drive>::SharedPtr trexPublisher;
    int maxSpeed = 140;
    int maxTurningSpeed = 90;
    int counter = 0;
};