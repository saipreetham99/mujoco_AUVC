#ifndef AUVC_SERIAL_PORT
#define AUVC_SERIAL_PORT
#include <string>
#include <termios.h>

struct SerialPort_ {             // Hold all information for AR TAGS
  // enable flags
  int serialPort;                 // Port Number
  struct termios port_settings;   // Terminal Info
  speed_t baud;                   // Baudrate
};
typedef struct SerialPort_ SerialPort;

int open_Serial_Port(SerialPort* serial, const char* portNum, int rate);

// read a single character
int read_Serial_Port(SerialPort* serial);

// read until special character up to a maximum number of bytes
std::string readBytesUntil(SerialPort* serial, unsigned char until, int max_length);

// send a string
void print_Serial_Port(SerialPort* serial, std::string str);

// send an integer
void print_Serial_Port(SerialPort* serial, int num);

// send a double
void print_Serial_Port(SerialPort* serial, double num);

// send a float
void print(SerialPort* serial, float num);

#endif // AUVC_SERIAL_PORT
