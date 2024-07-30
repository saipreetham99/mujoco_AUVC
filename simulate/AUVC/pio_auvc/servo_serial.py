import serial
import time



# Configuration for serial communication
SERIAL_PORT = '/dev/ttyACM0'  # Replace with your Teensy's serial port
BAUD_RATE = 9600
TIMEOUT = 1  # Timeout in seconds

def initialize_serial_connection(port, baud_rate):
    """Initialize the serial connection to the Teensy."""
    ser = serial.Serial(port, baud_rate, timeout=TIMEOUT)
    time.sleep(2)  # Give time for the connection to be established
    return ser

def send_motor_command(ser, command, value):
    """
    Send a command and a value to the Teensy.
    X is 0 value between -400 back and 400 front
    Y is 1 and same range for left and right
    Z is 2 for up and down.
    """
    ser.write(f'{command}\n'.encode())  # Send command
    time.sleep(0.1)  # Small delay
    ser.write(f'{value}\n'.encode())    # Send value

def main():
    ser = initialize_serial_connection(SERIAL_PORT, BAUD_RATE)

    try:
        while True:
            print("Enter command (0 for x, 1 for y, 2 for z) and value between -400 and 400:")
            user_input = input().strip()
            
            if user_input.lower() == 'exit':
                print("Exiting...")
                break

            parts = user_input.split()
            if len(parts) == 2:
                command = int(parts[0])
                value = int(parts[1])
                send_command(ser, command, value)
            else:
                print("Invalid input format. Enter command and value separated by space.")
            
    except KeyboardInterrupt:
        print("\nExiting due to keyboard interrupt...")
    
    finally:
        ser.close()

if __name__ == "__main__":
    main()

