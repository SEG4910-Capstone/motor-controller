#include "serialib.h"
#include <stdexcept> // For std::runtime_error
#include <string>
#include <iostream>
#include <cstdint>
using namespace std;

class Hdc2460Comms
{
public:
    Hdc2460Comms(const char *port, uint32_t timeout)
        : port(port), timeout(timeout), baudrate(115200) {}
        
    Hdc2460Comms() = default;

   void connect(const char *port, uint32_t timeout) {
      this->port = port;
      this->timeout = timeout;
      char errorOpening = serial.openDevice(port, 115200);
      if (errorOpening != 1) {
        is_connected = false;
      } else {
          is_connected = true;
      }

   }

   void connect() {
          char errorOpening = serial.openDevice(port, 115200);
          if (errorOpening != 1) {
            is_connected = false;
          } else {
            is_connected = true;
          }
    }

    void disconnect() {
        serial.closeDevice();
    }

    bool isConnected() {
        return is_connected; 
    } 

    std::string writeRead(const std::string& command) {
        std::string response;

        // Write the command followed by a carriage return
        std::string fullCommand = command + "\r";
        serial.writeString(fullCommand.c_str());

        // Read the response
        unsigned char buffer[256]; // Adjust size as needed
        int bytesRead = serial.readBytes(buffer, sizeof(buffer), timeout);

        if (bytesRead > 0) {
            response.assign(buffer, buffer + bytesRead);
        } else {
            return "Error or timeout";
        }

        return response;
    }

    std::string setMotor1Speed(const int speed) {
        std::string command = "!G 1 " + std::to_string(speed);
        return writeRead(command);
    }

    std::string setMotor2Speed(const int speed) {
        std::string command = "!G 2 " + std::to_string(speed);
        return writeRead(command);
    }

    std::string readEncoders() {
        std::string command = "?C 1";
        return writeRead(command);
    }

private:
    serialib serial;
    const char *port;
    int32_t baudrate;
    uint32_t timeout;
    
    bool is_connected;
};
