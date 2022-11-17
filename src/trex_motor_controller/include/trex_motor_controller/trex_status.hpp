#pragma once

#include <cstdint>
#include "trex_error_flags.hpp"
#include "motor_direction.hpp"

namespace TRex {

  // enum class StatusFlags {
  //   OK          = 0x00,
  //   TIMEOUT     = 0x01,
  //   BATTERY_LOW = 0x02,
  // };

  struct Status {
    // TODO: Status Flags

    double batteryVoltage = 0;

    uint8_t leftMotorSpeed = 0;
    MotorDirection leftMotorDirection = MotorDirection::FORWARD;
    int16_t leftMotorCurrent = 0;
    bool leftMotorBraking = false;

    uint8_t rightMotorSpeed = 0;
    MotorDirection rightMotorDirection = MotorDirection::FORWARD;
    int16_t rightMotorCurrent = 0;
    bool rightMotorBraking = false;

    // TODO: Error Flags

  };

}

  // uint8_t buffer[] = {
  //   STATUS_PACKET_START_BYTE,
  //   // Status Flags
  //   (uint8_t)(status.flags),
  //   // Battery
  //   highByte(batteryVoltage),
  //   lowByte(batteryVoltage),
  //   // Left Motor
  //   status.leftMotor.speed,
  //   (uint8_t)status.leftMotor.direction,
  //   highByte(status.leftMotor.current),
  //   lowByte(status.leftMotor.current),
  //   (uint8_t)status.leftMotor.braking,
  //   // Right Motor
  //   status.rightMotor.speed,
  //   (uint8_t)status.rightMotor.direction,
  //   highByte(status.rightMotor.current),
  //   lowByte(status.rightMotor.current),
  //   (uint8_t)status.rightMotor.braking,
  //   // Errors
  //   // TODO: Add i2c errors and such
  //   0x00      // ERRORS = NONE
  // };
