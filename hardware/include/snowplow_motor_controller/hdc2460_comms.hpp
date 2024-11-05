#include "serialib.h"
#include <stdexcept> // For std::runtime_error
#include <string>
#include <iostream>
#include <cstdint>
#include <vector>
#include <regex>
#include <sstream>
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

    int readEncoderCh1() {
        std::string command = "?C 1";
        std::string response = writeRead(command);

        if (response.length() > 4) {
            response = response.substr(4, response.length());
        }

        std::vector<int> integers =  parseIntegers(response);

        if (!integers.empty()) {
            return integers[0];
        } else {
            return 0;
        }
    }

    int readEncoderCh2() {
        std::string command = "?C 2";
        std::string response = writeRead(command);

        if (response.length() > 4) {
            response = response.substr(4, response.length());
        }

        std::vector<int> integers =  parseIntegers(response);

        if (!integers.empty()) {
            return integers[0];
        } else {
            return 0;
        }
    }

    int extractIntegerWords(const std::string& str) {
        std::string temp;
        bool isNegative = false; // To handle negative numbers

        for (char ch : str) {
            // Check if the character is a digit
            if (std::isdigit(ch)) {
                temp += ch; // Build the numeric string
            } else if (ch == '-' && temp.empty()) {
                // Handle negative sign for the first digit
                isNegative = true;
            } else if (!temp.empty()) {
                // If we have collected digits and encounter a non-digit, break
                break;
            }
        }

        // If temp is not empty, convert to integer and return
        if (!temp.empty()) {
            int number = std::stoi(temp);
            return isNegative ? -number : number; // Return the integer, adjusting for negative
        }

        // If no integer found, return 0 (or handle as needed)
        return 0;
    }

    std::vector<int> parseIntegers(const std::string& input) {
        std::vector<int> integers;
        std::regex integerRegex("-?\\d+"); // Regex to match integers (including negative)
        std::smatch matches;

        // Use regex_search to find all matches in the input string
        std::string::const_iterator searchStart(input.cbegin());
        while (std::regex_search(searchStart, input.cend(), matches, integerRegex)) {
            integers.push_back(std::stoi(matches[0])); // Convert the matched string to int
            searchStart = matches.suffix().first; // Move the search start to the end of the last match
        }

        return integers;
    }

private:
    serialib serial;
    const char *port;
    int32_t baudrate;
    uint32_t timeout;
    
    bool is_connected;
};
