import serial
import time

# Define the serial ports for communication
ser_device1 = serial.Serial('/dev/ttyS4', baudrate=115200)
ser_device2 = serial.Serial('/dev/ttyAML3', baudrate=115200)

def get_local_time():
    return time.time()

def send_time_request(ser):
    ser.write(b"REQUEST")

def receive_time_response(ser):
    response = ser.read(8)  # Assuming the response is 8 bytes (64 bits)
    return int.from_bytes(response, byteorder='big', signed=False)

def update_local_time(new_time):
    global_offset = new_time - get_local_time()
    return get_local_time() + global_offset

if __name__ == "__main__":
    while True:
        # Device 1 sends a time request to Device 2
        send_time_request(ser_device1)

        # Device 2 receives the time request and sends its local time back
        received_time_request = ser_device2.read(7)  # Assuming request is 7 bytes
        if received_time_request == b"REQUEST":
            current_time = get_local_time()
            ser_device2.write(current_time.to_bytes(8, byteorder='big', signed=False))

        # Device 1 receives the time response and updates its local time
        received_time_response = ser_device1.read(8)  # Assuming response is 8 bytes
        new_time = int.from_bytes(received_time_response, byteorder='big', signed=False)
        updated_time = update_local_time(new_time)

        print("Device 1 local time:", get_local_time())
        print("Device 2 local time:", updated_time)

        # Wait for a while before repeating the synchronization
        time.sleep(10)

