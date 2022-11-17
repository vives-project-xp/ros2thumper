#pragma once

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <string>
#include <fcntl.h>          //O_RDWR
#include "trex_status.hpp"
#include <stdexcept>
#include "trex_command.hpp"
#include <mutex>
#include "motor_direction.hpp"

namespace TRex {

  /*
    * Pointer to write function used for i2c communication.
    * Also see http://linux.die.net/man/2/write
    */
  ssize_t (* _linux_write)(int, const void *, size_t) = write;
  
  /*
    * Pointer to read function used for i2c communication.
    * Also see http://linux.die.net/man/2/read
    */
  ssize_t (* _linux_read)(int, void *, size_t) = read;

  /*
    * Pointer to open function used for i2c communication.
    * Also see https://linux.die.net/man/2/open
    */
  int (* _linux_open)(const char *pathname, int flags, ...) = open;

  /*
    * Pointer to close function used for i2c communication.
    * Also see https://linux.die.net/man/2/open
    */
  int (* _linux_close)(int fd) = close;


  class TRexMotorController {

    public:
      TRexMotorController(std::string device = "/dev/i2c-1", uint8_t address = 0x30) {
        this->device = device;
        this->address = address;
      }

      bool open(void) {
        mutex.lock();
        fileDescriptor = _linux_open(device.c_str(), O_RDWR);

        int result = -1;
        if (fileDescriptor >= 0) {
          result = ioctl(fileDescriptor, I2C_SLAVE, address);
        }
        mutex.unlock();

        return (fileDescriptor >= 0 && result >= 0);
      }

      Status status(void) {
        uint8_t buffer[STATUS_PACKET_SIZE] = { 0 };
        
        mutex.lock();
        int result = _linux_read(fileDescriptor, buffer, STATUS_PACKET_SIZE);
        mutex.unlock();

        if (result < 0) throw std::runtime_error("Failed to read TRex status");
        if (buffer[0] != STATUS_START_BYTE) throw std::runtime_error("Failed to read TRex status - wrong startbyte");

        return Status{
          // TODO: Status Flags

          (double)((buffer[2] << 8) + buffer[3])/100,   // Battery

          (uint8_t)(buffer[4]),                         // Left Motor Speed
          (MotorDirection)(buffer[5]),                  // Left Motor Direction
          (int16_t)((buffer[6] << 8) + buffer[7]),       // Left Motor Current
          (bool)(buffer[8]),                            // Left Motor Braking

          (uint8_t)(buffer[9]),                         // Right Motor Speed
          (MotorDirection)(buffer[10]),                 // Right Motor Direction
          (int16_t)((buffer[11] << 8) + buffer[12]),     // Right Motor Current
          (bool)(buffer[13])                            // Right Motor Braking

          // TODO: Error Flags
        };
      }

      void drive(uint8_t leftSpeed, MotorDirection leftDirection, uint8_t rightSpeed, MotorDirection rightDirection) {
        // [ 0xBA 0x01 LeftDir LeftSpeed RightDir RightSpeed ]

        uint8_t buffer[COMMAND_PACKET_MAX_SIZE] = { 
          COMMAND_PACKET_START_BYTE,
          (uint8_t)(Command::DRIVE),
          (uint8_t)(leftDirection),
          (uint8_t)(leftSpeed),
          (uint8_t)(rightDirection),
          (uint8_t)(rightSpeed)
        };
        write_buffer(buffer, COMMAND_PACKET_MAX_SIZE);
      }

      void stop(void) {
        // [ 0xBA 0x02 ]

        uint8_t buffer[COMMAND_PACKET_MAX_SIZE] = { 
          COMMAND_PACKET_START_BYTE,
          (uint8_t)(Command::STOP)
        };
        write_buffer(buffer, COMMAND_PACKET_MAX_SIZE);
      }

    private:
      void write_buffer(uint8_t * buffer, size_t length) {
        mutex.lock();
        int result = _linux_write(fileDescriptor, buffer, length);
        mutex.unlock();

        if (result < 0) throw std::runtime_error("Failed to write TRex command");
      }

      bool close(void) {
        mutex.lock();
        int result = _linux_close(fileDescriptor);
        fileDescriptor = -1;
        mutex.unlock();
        return (result == 0);
      }

    private:
      std::string device = "/dev/i2c-1";
      uint8_t address = 0x30;
      int fileDescriptor = -1;

      const static int STATUS_PACKET_SIZE = 15;
      const static int STATUS_START_BYTE = 0xAB;

      const static int COMMAND_PACKET_MAX_SIZE = 6;
      const static int COMMAND_PACKET_START_BYTE = 0xBA;
      std::mutex mutex;

      Command currentCommand;

  };

}
