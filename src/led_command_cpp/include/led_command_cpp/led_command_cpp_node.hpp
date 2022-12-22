#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "led_command_interface/msg/led_command.hpp"
#include <cmath>

class LedCommandCppNode : public rclcpp::Node {
  public:
    LedCommandCppNode() : Node("led_command_cpp_node") {
      RCLCPP_INFO(this->get_logger(), "Starting led command Node ...");

      joySubscriber = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy",          // topic
        10,              // The depth of the subscription's incoming message queue.
        std::bind(&LedCommandCppNode::joy_callback, this, std::placeholders::_1)   // callback takes 1 arg
      );

      ledValue = this->create_publisher<led_command_interface::msg::LedCommand>("ledValue", 10);
    }

  private:
    void joy_callback(sensor_msgs::msg::Joy::UniquePtr msg) {
      // TODO: Need to drop some messages. Joy is publishing to fast. Tried topic_tools drop but can't build it.
      // Crude solution for getting this to work
      if ((++counter) % 4 != 0) return;
    
      auto message = led_command_interface::msg::LedCommand();

      RCLCPP_INFO(this->get_logger(), "Processing Joystick Message");

      if (msg->buttons[0] != prevFaars){
        toggleBool(&faars);
        prevFaars = msg->buttons[0];
      }

      if (msg->buttons[2] != prevZwaaiLicht){
        toggleBool(&zwaailicht);
        prevZwaaiLicht = msg->buttons[2];
      }

      if (msg->buttons[4] != prevPinkerLinks){
        toggleBool(&pinkerLinks);
        prevPinkerLinks = msg->buttons[4];
      }

      if (msg->buttons[5] != prevPinkerRechts){
        toggleBool(&pinkerRechts);
        prevPinkerRechts = msg->buttons[5];
      }

      double leftJoystick = msg->axes[1];
      double rightJoystick = msg->axes[4];

      if (leftJoystick <= 0.1 && rightJoystick <= 0.1){
        stop = true;
      }else{
        stop = false;
      }

      int voorLinks[12];
      int voorRechts[12];
      int achterLinks[12];
      int achterRechts[12];

      for (size_t i = 0; i < 12; i++)
      {
        voorLinks[i] = WHITE;
      }

      for (size_t i = 0; i < 12; i++)
      {
        voorRechts[i] = WHITE;
      }

      for (size_t i = 0; i < 12; i++)
      {
        achterLinks[i] = RED;
      }

      for (size_t i = 0; i < 12; i++)
      {
        achterRechts[i] = RED;
      }
      
      if(stop){
        for (size_t i = 0; i < 12; i++){
          achterLinks[i] = BRIGHT_RED;
          achterRechts[i] = BRIGHT_RED; 
        }
      }

      if(faars){
        for (size_t i = 0; i < 12; i++){
          voorLinks[i] = BRIGHT_WHITE;
          voorRechts[i] = BRIGHT_WHITE; 
        }
      }

      if(pinkerLinks){
        int color;
        if(pinkerAanUit){
          color = ORANGE;
        }else{
          color = BLACK;
        }
        toggleBool(&pinkerAanUit);

        for (size_t i = 0; i < 6; i++){
          voorLinks[i] = color;
          achterLinks[i] = color;
        }
      }

      if(pinkerRechts){
        int color;
        if(pinkerAanUit){
          color = ORANGE;
        }else{
          color = BLACK;
        }
        toggleBool(&pinkerAanUit);

        for (size_t i = 0; i < 6; i++){
          voorRechts[i] = color;
          achterRechts[i] = color;
        }
      }

      if (zwaailicht){
        for (size_t i = 0; i < 12; i++){
          
            if(i % 2){
              if(zwaaiLichtState){
                zwaaiLicht[i] = ORANGE;
              }else{
                zwaaiLicht[i] = BLACK;
              }
            }else{
              if(zwaaiLichtState){
                zwaaiLicht[i] = BLACK;
              }else{
                zwaaiLicht[i] = ORANGE;
              }
            }
        }
        toggleBool(&zwaaiLichtState);
      }

      std::string tmp;

      for (size_t i = 0; i < 12; i++){
        message.front_left[i] = voorLinks[i];
      }
      
      for (size_t i = 0; i < 12; i++){
        message.front_right[i] = voorRechts[i];
      }

      for (size_t i = 0; i < 12; i++){
        message.back_left[i] =achterLinks[i];
      }

      for (size_t i = 0; i < 12; i++){
        message.back_right[i] = achterRechts[i];
      }

      for (size_t i = 0; i < 12; i++){
        message.hazerd_light[i] = zwaaiLicht[i];
      }

      

      ledValue->publish(message);
    }

    void toggleBool(bool * var){
      *var = ! (*var);
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySubscriber;
    rclcpp::Publisher<led_command_interface::msg::LedCommand>::SharedPtr ledValue;
    int counter = 0;
    bool faars = false;
    bool pinkerLinks = false;
    bool pinkerRechts = false;
    bool zwaailicht = false;
    bool pinkerAanUit = true;
    bool zwaaiLichtState = true;
    bool stop = false;

    int prevFaars = 0;
    int prevPinkerLinks = 0;
    int prevPinkerRechts = 0;
    int prevZwaaiLicht = 0;

    const int ORANGE = 0x301800;
    const int RED = 0x300000;
    const int BRIGHT_RED = 0x600000;
    const int WHITE = 0x202020;
    const int BRIGHT_WHITE = 0x606060;
    const int BLACK = 0x000000;

    int zwaaiLicht[12] = {0x0};
};