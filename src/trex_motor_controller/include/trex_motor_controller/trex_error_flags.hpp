#pragma once

#include <unistd.h>

namespace TRex {

  enum class ErrorFlags {

    NONE            = 0x00,
    START_BYTE      = 0x01,     // Start byte not received or incorrect data packet size.
    COMMAND_SIZE    = 0x02,     // Number of bytes received is not correct
    MOTOR_SPEED     = 0x04,     // Left or right motor speed was not -255 to +255.
    BATT_THRESHOLD  = 0x08,     // Low battery was not 550 to 3000 (5.5V to 30V).

  };

}