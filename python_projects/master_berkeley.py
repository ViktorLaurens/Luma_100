import serial
import time
import sys
from timeit import default_timer as timer

agent_id = int(sys. argv[1])
time_received = False
time_offset = 0

def send_time_request(master):
    master.write(b"REQUEST")

def receive_time_response(master):
    response = master.read(8)  # Assuming the response is 8 bytes (64 bits)
    return int.from_bytes(response, byteorder='big', signed=False)

def update_time_offset(slave_time, time_offset):
    master_time = int(time.time() * 1_000_000) + time_offset
    offset = slave_time - master_time
    return time_offset + offset

if __name__ == "__main__":
    if agent_id == 0:
        master = serial.Serial('/dev/ttyS4', baudrate=115200)
    elif agent_id == 1:
        master = serial.Serial('/dev/ttyAML3', baudrate=115200)
    while True:
        send_time_request(master)
        request_time = timer()
        print('REQUEST sent')
        if master.in_waiting > 0:
            slave_time = receive_time_response(master)
            response_time = timer()
            process_delay_latency = response_time - request_time
            slave_time = slave_time + process_delay_latency * 1_000_000 / 2
            current_time = int(time.time() * 1_000_000) + time_offset
            print('process latency:', process_delay_latency * 1_000_000 / 2)
            if slave_time < current_time:
                time_offset = update_time_offset(slave_time, time_offset)
            print('time offset:', time_offset)
            print('time_diff:', current_time - slave_time)
            time_received = True
        if time_received: 
            # Wait for a while before repeating the synchronization
            time.sleep(2)
            master.flushInput()
            time_received = False
        time.sleep(0.1)
