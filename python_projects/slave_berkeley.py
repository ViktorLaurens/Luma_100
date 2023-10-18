import serial
import time
import sys

agent_id = int(sys. argv[1])
time_sent = False
local_time = int(time.time() * 1_000_000)

if __name__ == "__main__":
    # Define whether this device is master or slave
    if agent_id == 0:
        slave = serial.Serial('/dev/ttyS4', baudrate=115200)
    elif agent_id == 1:
        slave = serial.Serial('/dev/ttyAML3', baudrate=115200)
    # Device 2 receives the time request and sends its local time back
    while True:
        message = slave.read(7)
        if message == b"REQUEST": 
            print('REQUEST received')
            local_time = int(time.time() * 1_000_000)
            slave.write(local_time.to_bytes(8, byteorder='big', signed=False))
            time_sent = True
            print("Sent:", local_time)
        else: 
            slave.flushInput()
