import serial, time, os, dccl,  protobuf_navreport_pb2, struct, sys, random

baud_rate = 115200

agent_id = int(sys.argv[1])

header_length = struct.calcsize('I')  # Calculate the size of the header (4 bytes)

# TDMA
time_slot_duration = 1  # Each time slot duration
num_agents = 2
message_written_flag = False 
header_received = False
message_length = 0

# Functions
def send_message():
    report = protobuf_navreport_pb2.StatusReport(x = 450, y=550, z=-100, veh_class=protobuf_navreport_pb2.StatusReport.AUV, battery_ok=True)  # Create an instance of the StatusReport message
    encoded_report = report.SerializeToString()
    print(encoded_report)
    message_length = len(encoded_report)
    header = struct.pack('I', message_length) # Pack the message length as a 4-byte integer (unsigned int)
    message_with_header = header + encoded_report # Prepend the header to the encoded message
    ser.write(message_with_header) # Write the entire message (header + encoded message) to the serial port (TX pin)

def send_random_message():
    report = protobuf_navreport_pb2.StatusReport(x=random.randint(-1000, 1000), y=random.randint(-1000, 1000), z=random.randint(-1000, 1000), veh_class=protobuf_navreport_pb2.StatusReport.AUV, battery_ok=True)
    encoded_report = report.SerializeToString()
    print(encoded_report)
    message_length = len(encoded_report)
    header = struct.pack('I', message_length) # Pack the message length as a 4-byte integer (unsigned int)
    message_with_header = header + encoded_report # Prepend the header to the encoded message
    ser.write(message_with_header) # Write the entire message (header + encoded message) to the serial port (TX pin) 

def receive_message():
    global header_received  # Declare the global variable
    global message_length

    if ser.in_waiting >= header_length and not header_received:
        header_received = True
        received_header = ser.read(header_length)  # Read the header
        message_length = struct.unpack('I', received_header)[0] # Unpack the received header to get the message length

    if header_received:
        if ser.in_waiting >= message_length:
            received_message = ser.read(message_length) # Read the remaining message based on the extracted message length
            try:
                decoded_msg = protobuf_navreport_pb2.StatusReport()
                decoded_msg.ParseFromString(received_message)
                print(decoded_msg)
            except protobuf_navreport_pb2.Error as e:
                print("Failed to decode:", e)
                ser.flushInput()
            header_received = False
            
#if __name__ == '__main__':

# Define the serial port
if agent_id == 0:
    ser = serial.Serial('/dev/ttyS4', baud_rate)
elif agent_id == 1:
    ser = serial.Serial('/dev/ttyAML3', baud_rate)

try:
    while True:
        current_time = time.time()
        current_time_slot = int(current_time / time_slot_duration) % num_agents
        if current_time_slot == agent_id and not message_written_flag:
            send_random_message()
            message_written_flag = True
        if current_time_slot != agent_id:
            if message_written_flag: 
                message_written_flag = False
            if ser.in_waiting > 0:
                receive_message()

except KeyboardInterrupt:
    print("\nExiting...")

finally:
    # Close the serial port
    ser.close()
