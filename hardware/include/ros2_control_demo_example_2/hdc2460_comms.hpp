
//#include <serial/serial.h>
#include <libserial/SerialPort.h>
#include <stdarg.h>

#include <cstdio>
#include <string>

using namespace std;

class Hdc2460Comms
{

public:

    /**
     * Constructor for the Hdc2460Comms class.
     *
     * @param port the serial port to use for communication
     * @param baudrate the baud rate for the serial connection
     * @param timeout the timeout value for the serial connection
     * 
     */
    // 
    Hdc2460Comms() = default;

    void connect(std::string port, uint32_t timeout) {
        m_serial.Open(port);
        m_serial.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        this->port = port;
        this->timeout = timeout;
    }

    void disconnect() {
        m_serial.Close();
    }

    bool isConnected() {
        return m_serial.IsOpen();
    }

    /**
     * Sends a command over the serial connection and waits for a response.
     *
     * @param command the command to send over the serial connection
     *
     * @return true if a response was received, false otherwise
     *
     * @throws None
     */
    std::string writeRead(const std::string command ) {

        std::string response = "";
        m_serial.FlushIOBuffers();
;
        m_serial.Write(stringFormat(command + "\r"));

        try {
          m_serial.Read(response, 8,this->timeout); 
        } catch (LibSerial::ReadTimeout&) {
            return "Error";
        }

        return response;
    }

    /**
     * Sets the speed of the motor on CHANNEL 1.
     *
     * @param speed The desired speed of the motor, range between 0 to 1000.
     *
     * @return `true` if the motor speed was successfully set, `false` otherwise.
     *
     * @throws None
     */
    std::string setMotor1Speed(const int speed) {
           
      m_serial.FlushIOBuffers();

      std::string command = "!G 1 " + std::to_string(speed);

      return writeRead(command);
    }

    std::string setMotor2Speed(const int speed) {
        
      m_serial.FlushIOBuffers();

      std::string command = "!G 2 " + std::to_string(speed);

      return writeRead(command);
    }

    std::string readEncoders() {
      
      m_serial.FlushIOBuffers();

      std::string command = "?S";

      return writeRead(command);
    }
    

/**
 * @brief format command string into the format roboteq requires
 * @param fmt const string
 * @return formated string
 */
inline std::string stringFormat(const std::string& fmt, ...)
{
  int size = 100;
  std::string str;
  va_list ap;
  while (1)
  {
    str.resize(size);
    va_start(ap, fmt);
    int n = vsnprintf((char*)str.c_str(), size, fmt.c_str(), ap);
    va_end(ap);
    if (n > -1 && n < size)
    {
      str.resize(n);
      return str;
    }
    if (n > -1)
    {
      size = n + 1;
    }
    else
    {
      size *= 2;
    }
  }
  return str;
}


private:
    LibSerial::SerialPort m_serial;

    std::string port;
    int32_t baudrate;
    uint32_t timeout;


};


