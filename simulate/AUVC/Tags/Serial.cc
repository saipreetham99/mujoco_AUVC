#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <sstream>
#include <unistd.h>

#include"Serial.h"

int open_Serial_Port(SerialPort* serial, const char* portNum, int rate) {
  serial->serialPort = ::open(portNum, O_RDWR | O_NOCTTY | O_NDELAY);
  if (serial->serialPort==-1) {
    printf("Unable to open serial port\n");
    exit(1);
  }
  fcntl(serial->serialPort, F_SETFL,0); // O_NONBLOCK might be needed for write...

  tcgetattr(serial->serialPort, &serial->port_settings); // get current settings

  switch(rate) {
  case(9600):
    serial->baud = B9600;
    break;
  case(19200):
    serial->baud = B19200;
    break;
  case(38400):
    serial->baud = B38400;
    break;
  case(115200):
    serial->baud = B115200;
    break;
  default:
    printf("Error: Unknown baud rate requested in Serial.open()\n");
    return 1;
  }

  cfsetispeed(&serial->port_settings, serial->baud);    // set baud rates
  cfsetospeed(&serial->port_settings, serial->baud);

  serial->port_settings.c_cflag &= ~PARENB;    // set no parity, stop bits, 8 data bits
  serial->port_settings.c_cflag &= ~CSTOPB;
  serial->port_settings.c_cflag &= ~CSIZE;
  serial->port_settings.c_cflag |= CS8;

  tcsetattr(serial->serialPort, TCSANOW, &serial->port_settings);    // apply the settings to the port
  return 0;
}

// read a single character
int read_Serial_Port(SerialPort* serial) {
  unsigned char result;
  if (::read(serial->serialPort, &result, 1) == 1) {
    return (int)result;
  } else {
    return -1;
  }
}

// read until special character up to a maximum number of bytes
std::string readBytesUntil(SerialPort* serial, unsigned char until, int max_length) {
  std::string result(max_length, ' ');
  int n = 0;
  int c;
  do {
    c = read_Serial_Port(serial);
    if (c<0) { // wait for more characters
      usleep(100);
    } else {
      result[n] = (unsigned char)c;
      n++;
    }
  } while ((c != (int)until) && (n < max_length));
  result.resize(n);
  return result;
}

// send a string
void print_Serial_Port(SerialPort* serial, std::string str) {
  int res = write(serial->serialPort, str.c_str(), str.length());
}

// send an integer
void print_Serial_Port(SerialPort* serial, int num) {
  std::stringstream stream;
  stream << num;
  std::string str = stream.str();
  print_Serial_Port(serial, str);
}

// send a double
void print_Serial_Port(SerialPort* serial, double num) {
  std::stringstream stream;
  stream << num;
  std::string str = stream.str();
  print_Serial_Port(serial,str);
}

// send a float
void print_Serial_Port(SerialPort* serial, float num) {
  print_Serial_Port(serial,(double)num);
}
