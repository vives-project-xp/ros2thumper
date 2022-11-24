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
      double speedMotorRight = forward * maxSpeed - turning * maxTurningSpeed;
      double speedMotorLeft = forward * maxSpeed + turning * maxTurningSpeed;

      if (std::abs(speedMotorLeft) > 20) {
        message.left_motor_speed = (uint8_t)(std::abs(speedMotorLeft));
      }
      if (std::abs(speedMotorRight) > 20) {
        message.right_motor_speed = (uint8_t)(std::abs(speedMotorRight));
      }

      if (speedMotorLeft < 0) {
        message.left_motor_direction = 1;
      } else {
        message.left_motor_direction = 0;
      }

      if (speedMotorRight < 0) {
        message.right_motor_direction = 1;
      } else {
        message.right_motor_direction = 0;
      }





      // if (forward > 0.1) {
      //   message.left_motor_speed = (uint8_t)(forward * maxSpeed);
      //   message.left_motor_direction = 1;    // Forward
      //   message.right_motor_speed = (uint8_t)(forward * maxSpeed);
      //   message.right_motor_direction = 1;    // Forward
      // } else if (forward < -0.1) {
      //   message.left_motor_speed = (uint8_t)(std::abs(forward * maxSpeed));
      //   message.left_motor_direction = 0;    // Backwards
      //   message.right_motor_speed = (uint8_t)(std::abs(forward * maxSpeed));
      //   message.right_motor_direction = 0;    // Backwards
      // } else {
      //   message.left_motor_speed = 0;
      //   message.left_motor_direction = 0;
      //   message.right_motor_speed = 0;
      //   message.right_motor_direction = 0;
      // }

      // if (turning > 0.1) { //Right
      //   message.left_motor_speed = (uint8_t)(maxSpeed);
      //   message.left_motor_direction = 0;    // Forward 
      //   message.right_motor_speed = (uint8_t)((1 - turning) * maxSpeed);  
      //   message.right_motor_direction = 0;
      // } else if (turning < -0.1) {  //Left
      //   message.left_motor_speed = (uint8_t)(std::abs((1 - turning) * maxSpeed));
      //   message.left_motor_direction = 0;    // Backwards
      //   message.right_motor_speed = (uint8_t)(std::abs(maxSpeed));
      //   message.right_motor_direction = 0;    // Backwards
      // }

      // if (rightJoystick > 0.1) {
      //   message.right_motor_speed = (uint8_t)(rightJoystick * 255);
      //   message.right_motor_direction = 0;    // Forward
      // } else if (rightJoystick < -0.1) {
      //   message.right_motor_speed = (uint8_t)(std::abs(rightJoystick * 255));
      //   message.right_motor_direction = 1;    // Backwards
      // } else {
      //   message.right_motor_speed = 0;
      //   message.right_motor_direction = 0;
      // }

      trexPublisher->publish(message);
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySubscriber;
    rclcpp::Publisher<trex_interfaces::msg::Drive>::SharedPtr trexPublisher;
    int maxSpeed = 100;
    int maxTurningSpeed = 50;
    int counter = 0;
};