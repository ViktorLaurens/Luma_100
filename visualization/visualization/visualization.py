import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Point
from custom_interfaces.msg import NavigationReport
from rclpy.qos import ReliabilityPolicy, QoSProfile
from collections import deque 
import numpy as np

# Constants for the number of bits allocated for agent ID and message ID
AGENT_ID_BITS = 3  # Adjust based on the number of agents (e.g., 2, 3, 4, ...)
MESSAGE_ID_BITS = 9  # Adjust based on the expected number of messages (e.g., 256, 512, ...)

class Visualization(Node):
    def __init__(self, ):
        super().__init__('visualization')
        # Initialize parameters, publishers, subscribers, and other variables
        self.init_parameters()
        self.init_publishers()
        self.init_subscribers()

    def init_parameters(self):
        # Initialize message queue for the sent messages
        #self.send_message_queue = deque()
        # Initialize message queue for the received messages  
        #self.rec_message_queue = deque()
        self.sent_ids = []
        self.received_ids = {}

    def init_publishers(self):
        # Create publishers for sending messages
        pass

    def init_subscribers(self):
        # Create subscribers for receiving messages
        self.OwnNavReportSubscriber = self.create_subscription(NavigationReport, 'own_nav_report_0', self.nav_callback, 10)
        self.visRecSubscriber = self.create_subscription(NavigationReport, 'received_0', self.received_callback, 10)

    def nav_callback(self, msg):
        # Add the received message to the send_message_queue
        #self.send_message_queue.append(msg)
        # Extract the data for plotting
        agent_id, message_counter = self.extract_from_message_id(msg.message_id)
        self.sent_ids.append(message_counter)
        # Formatting the log message
        log_message = '---'.join(map(str, self.sent_ids))
        self.get_logger().info(f'Robot {agent_id} sent: {log_message}\n')

    def received_callback(self, msg):
        agent_id, message_counter = self.extract_from_message_id(msg.message_id)
        # Check if agent_id is already a key in the dictionary
        if agent_id not in self.received_ids:
            self.received_ids[agent_id] = []
        self.received_ids[agent_id].append(message_counter)
        # Formatting the log message
        log_message = '---'.join(map(str, self.received_ids[agent_id]))
        self.get_logger().info(f'Received from robot {agent_id}: {log_message}\n')


    def extract_from_message_id(self, message_id):
        # Extract message_counter from the lower MESSAGE_ID_BITS bits of message_id
        message_counter = message_id & ((1 << MESSAGE_ID_BITS) - 1)
        # Extract agent_id by right-shifting the message_id
        agent_id = message_id >> MESSAGE_ID_BITS
        return agent_id, message_counter

def main(args=None):
    rclpy.init(args=args)
    visualization = Visualization()
    rclpy.spin(visualization)
    visualization.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
