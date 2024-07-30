#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>  // For strlen

int main() {
    const char *port = "/dev/ttyACM0";  // Replace with the correct port name
    int serial_port = open(port, O_RDWR | O_NOCTTY | O_NDELAY);

    if (serial_port < 0) {
        std::cerr << "Error opening serial port." << std::endl;
        return 1;
    }

    // Configure serial port
    termios tty;
    if (tcgetattr(serial_port, &tty) != 0) {
        std::cerr << "Error from tcgetattr." << std::endl;
        close(serial_port);
        return 1;
    }

    cfsetispeed(&tty, B9600); // Input baud rate
    cfsetospeed(&tty, B9600); // Output baud rate

    tty.c_cflag |= (CLOCAL | CREAD); // Enable receiver and set local mode
    tty.c_cflag &= ~CSIZE; // Mask character size bits
    tty.c_cflag |= CS8; // 8 data bits
    tty.c_cflag &= ~PARENB; // No parity bit
    tty.c_cflag &= ~CSTOPB; // 1 stop bit
    tty.c_cflag &= ~CRTSCTS; // No hardware flow control

    tcsetattr(serial_port, TCSANOW, &tty); // Apply the settings

    // Write and echo loop
    const char *message = "Hello Teensy!";
    char buf[256];
    bool messageSent = false;

    while (true) {
        if (!messageSent) {
            // Write a message to Teensy
            write(serial_port, message, strlen(message));
            messageSent = true;
        }

        // Read from serial port
        int num_bytes = read(serial_port, buf, sizeof(buf) - 1);
        if (num_bytes < 0) {
            std::cerr << "Error reading from serial port." << std::endl;
            close(serial_port);
            return 1;
        }

        if (num_bytes > 0) {
            buf[num_bytes] = '\0'; // Null-terminate the string
            std::cout << "Received: " << buf << std::endl;
            
            // Echo the received data back to Teensy
            write(serial_port, buf, num_bytes);
        }

        usleep(100000); // Sleep for 100 milliseconds to prevent busy waiting
    }

    close(serial_port);
    return 0;
}

