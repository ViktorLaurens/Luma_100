import os
import sys
import time
import struct
import serial
import dccl
import rclpy

from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from custom_interfaces.msg import NavigationReport
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray

directory_path = 'src/opt_comm/opt_comm'
sys.path.append(directory_path)
dccl.loadProtoFile(os.path.abspath("src/opt_comm/opt_comm/navreport.proto"))
codec = dccl.Codec()
codec.load("NavigationReport")

import navreport_pb2

class TDMACommunication(Node):
    HEADER_LENGTH = struct.calcsize('HH')
    PORT = '/dev/ttyS4'

    def __init__(self):
        super().__init__('tdma_node')
        self._load_parameters()
        self._initialize_serial_communication()
        self._initialize_publishers()
        self._initialize_subscribers(self.callback_group)
        self._initialize_timer(0.2, self.tdma_loop, self.callback_group)

    def _load_parameters(self): 
        self.declare_parameters(
            namespace='',
            parameters=[
                ('baud_rate', rclpy.Parameter.Type.INTEGER),
                ('agent_id', rclpy.Parameter.Type.INTEGER),
                ('time_slot_duration', rclpy.Parameter.Type.DOUBLE),
                ('num_agents', rclpy.Parameter.Type.INTEGER),
                ('message_written_flag', rclpy.Parameter.Type.BOOL),
                ('header_received', rclpy.Parameter.Type.BOOL),
                ('message_length', rclpy.Parameter.Type.INTEGER),
                ('ack_received', rclpy.Parameter.Type.BOOL),
                ('sequence_number', rclpy.Parameter.Type.INTEGER),
                ('previous_ack_id', rclpy.Parameter.Type.INTEGER),
                ('ack_id', rclpy.Parameter.Type.INTEGER)
            ]
        )

        params = [
            'baud_rate', 'agent_id', 'time_slot_duration', 'num_agents', 
            'message_written_flag', 'header_received', 'message_length',
            'ack_received', 'sequence_number', 'previous_ack_id', 'ack_id'
        ]
        self.get_parameters(params)
        
        for param in params:
            setattr(self, param, self.get_parameter(param).value)   
        
    def _initialize_serial_communication(self):
        self.ser = serial.Serial(self.PORT, self.baud_rate)
        self.msg_to_transmit = NavigationReport()
        #self.request_made = False
        self.message_received = False
        self.comm_man_updated = False
        self.comm_man_finished_updating = False
        self.msg_ids_refreshed = False
        self.callback_group = ReentrantCallbackGroup()
        self.received_message_ids = Int16MultiArray()

    def _initialize_publishers(self):
        #qos = QoSProfile(reliability=ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
        self.sendPublisher = self.create_publisher(NavigationReport, 'msg_sent_0', 10)
        self.reqPublisher = self.create_publisher(String, 'msg_request_0', 10)
        self.receivePublisher = self.create_publisher(NavigationReport, 'msg_received_0', 10)
        # A publisher to send the received message_ids
        self.msgIdPublisher = self.create_publisher(Int16MultiArray, 'received_message_ids_0', 10)

    def _initialize_subscribers(self, callback_group):
        #qos = QoSProfile(reliability=ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
        self.msgSubscriber = self.create_subscription(NavigationReport, 'msg_to_transmit_0', self.receive_msg_to_transmit, 10, callback_group=callback_group)
        self.commManUpdatedSubscriber = self.create_subscription(Bool, 'comm_man_updated_0', self.comm_man_update, 10, callback_group=callback_group)

    def _initialize_timer(self, timer_period, loop_function, callback_group):
        self.timer = self.create_timer(timer_period, loop_function, callback_group)

    def request_message_to_transmit(self):
        request = String(data='message request')
        self.reqPublisher.publish(request)
        #self.get_logger().info('request_message_to_transmit')

    def receive_msg_to_transmit(self, msg):
        self.msg_to_transmit = msg
        self.message_received = True
        #self.get_logger().info('receive_msg_to_transmit')

    def comm_man_update(self, msg):
        self.comm_man_finished_updating = msg.data
    
    def transmit_message(self, navigation_report):
        # Publish navigation report 
        msg = self._convert_to_navigation_report(navigation_report)
        self.sendPublisher.publish(msg)
        self.get_logger().info(f'Publishing NavReport: ID:{msg.message_id}, ACK:{msg.ack_ids}')
        # Send out the message through UART
        self._write_to_uart(msg)

    def _convert_to_navigation_report(self, navigation_report):
        veh_classes = [navreport_pb2.NavigationReport.AUV, navreport_pb2.NavigationReport.USV, navreport_pb2.NavigationReport.SHIP]
        return NavigationReport(
            message_id=navigation_report.message_id,
            ack_ids=navigation_report.ack_ids,
            x=navigation_report.x,
            y=navigation_report.y,
            z=navigation_report.z,
            veh_class=veh_classes[navigation_report.veh_class - 1],
            battery_ok=navigation_report.battery_ok
        )
    
    def _write_to_uart(self, msg):
        r_out = navreport_pb2.NavigationReport(x=msg.x, y=msg.y, z=msg.z, veh_class=msg.veh_class, battery_ok=msg.battery_ok)
        encoded_bytes = codec.encode(r_out)
        
        # Create a header with the message_id, number of ack_ids, and length of the protobuf message
        ack_ids_length = len(msg.ack_ids)
        header_format = 'H' + 'H' + 'H' * ack_ids_length + 'H'  # One 'H' for message_id, one for the number of ack_ids, and then one for each ack_id
        header = struct.pack(header_format, msg.message_id, ack_ids_length, *msg.ack_ids, len(encoded_bytes))
    
        self.ser.write(header + encoded_bytes)

    def receive_message(self):
        if self.ser.in_waiting >= self.HEADER_LENGTH:
            header = struct.unpack('HH', self.ser.read(self.HEADER_LENGTH))
            message_id, num_ack_ids = header

            # Reading the ack_ids
            ack_id_format = 'H' * num_ack_ids
            ack_ids = list(struct.unpack(ack_id_format, self.ser.read(2 * num_ack_ids)))

            # Reading the message length
            self.message_length = struct.unpack('H', self.ser.read(2))[0]

            if self.ser.in_waiting >= self.message_length:
                try: 
                    decoded_msg = codec.decode(self.ser.read(self.message_length))
                    
                    # Create a NavigationReport message and populate it with decoded data
                    msg = NavigationReport()
                    msg.message_id = message_id
                    msg.ack_ids = ack_ids
                    msg.x = decoded_msg.x
                    msg.y = decoded_msg.y
                    msg.z = decoded_msg.z
                    msg.veh_class = decoded_msg.veh_class
                    msg.battery_ok = decoded_msg.battery_ok
                    
                    # Append the message_id to the list
                    self.received_message_ids.data.append(message_id)
                    
                    # Publish the NavigationReport message
                    self.receivePublisher.publish(msg)
                    self.get_logger().info(f'Reading NavReport: ID:{msg.message_id}, ACK:{msg.ack_ids}')
                except dccl.DcclException as e: 
                    self.ser.flushInput()
            else: 
                self.ser.flushInput()               
        else: 
            self.ser.flushInput()

    def update_comm_man(self):
        self.msgIdPublisher.publish(self.received_message_ids)
        self.get_logger().info(f'received_message_ids: {self.received_message_ids}')
        self.received_message_ids = Int16MultiArray()
    
    def tdma_loop(self): 
        current_time = time.time()
        current_time_slot = int(current_time / self.time_slot_duration) % self.num_agents   # Calculate if frame is for sending or receiving based on agent_id
        if current_time_slot == self.agent_id:
            # Before sending out a new message we update the communication manager with all the received data
            if not self.comm_man_updated:
                self.update_comm_man()
                self.ser.flushInput()
                self.comm_man_updated = True
                # reset other booleans
                self.msg_ids_refreshed = False
            while not self.comm_man_finished_updating: 
                pass
            self.request_message_to_transmit()
            while not self.message_received:
                pass  # This could be improved with a sleep or another mechanism to avoid busy waiting
            self.message_received = False
            self.transmit_message(self.msg_to_transmit)
        if current_time_slot != self.agent_id:
            if not self.msg_ids_refreshed:
                self.received_message_ids = Int16MultiArray()
                self.msg_ids_refreshed = True
                self.comm_man_updated = False
                self.comm_man_finished_updating = False
            if self.ser.in_waiting > 0:    # while??
                self.receive_message()

def main(args=None):
    rclpy.init(args=args)
    tdma_node = TDMACommunication()
    executor = MultiThreadedExecutor()   
    tdma_node.ser.flushInput()    
    rclpy.spin(tdma_node, executor)
    tdma_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

