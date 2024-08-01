import socket
import struct
import time

def send_data(host='192.168.2.2', port=8000, data=[1.0, 2.0, 3.0, 4.0]):
    # Convert list of floats to bytes
    byte_data = struct.pack(f'{len(data)}f', *data)
    # Prepare length of the data
    length = len(byte_data)
    
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((host, port))
        # Send the length of the data
        s.sendall(struct.pack('I', length))
        # Send the actual data
        s.sendall(byte_data)

if __name__ == "__main__":
    # Example data to send
    data = [1.5, 2.5, 3.5, 4.5]
    
    while True:
        send_data(data=data)
        print(f"Sent data: {data}")
        time.sleep(5)  # Wait for 5 seconds before sending data again

