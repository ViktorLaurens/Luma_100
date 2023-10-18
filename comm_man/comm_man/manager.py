import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Point
from custom_interfaces.msg import NavigationReport
from rclpy.qos import ReliabilityPolicy, QoSProfile
from collections import deque 

# Constants for the number of bits allocated for agent ID and message ID
AGENT_ID_BITS = 3  # Adjust based on the number of agents (e.g., 2, 3, 4, ...)
MESSAGE_ID_BITS = 9  # Adjust based on the expected number of messages (e.g., 256, 512, ...)

class CommunicationManager(Node):
    def __init__(self):
        super().__init__('communication_manager')

        # Initialize parameters, publishers, subscribers, and other variables
        self.init_parameters()
        self.init_publishers()
        self.init_subscribers()

    def init_parameters(self):
        # Declare parameters to load from YAML file
        self.declare_parameters(
            namespace='',
            parameters=[
                ('agent_id', rclpy.Parameter.Type.INTEGER),
                ('num_agents', rclpy.Parameter.Type.INTEGER),
                ('veh_class', rclpy.Parameter.Type.INTEGER), 
                ('battery_ok', rclpy.Parameter.Type.BOOL)
            ]
        )
        # Load parameters from the parameter server
        self.get_parameters(['agent_id',
                             'num_agents',
                             'veh_class', 
                             'battery_ok'
                            ]
                        )
        # Access the values of parameters
        self.agent_id = self.get_parameter('agent_id').value
        self.num_agents = self.get_parameter('num_agents').value
        self.veh_class = self.get_parameter('veh_class').value
        self.battery_ok = self.get_parameter('battery_ok').value
        # Declare other parameters
        self.message_counter = 1
        self.ack_ids_list = Int16MultiArray()
        # Initialize message queue for the navigation_data messages to transmit
        self.transmit_message_queue = deque()
        # Initialize message queue for the transmitted messages waiting for acknowledgement 
        self.ack_message_queue = deque()
        self.acknowledgment_tracking = {}  # The key is the message_id and the value is a set of agent_ids that have acknowledged the message
        # Initialize message queue for the received messages  
        self.rec_message_queue = deque()
        # Initialize message queue for the received acknowledgements
        self.ack_ids_queue = deque(maxlen=3)
        # Initialize sets
        self.rec_message_set = set()  # For quick look-up
        self.active_agents = set()   # To store active agents based on the ping messages

    def init_publishers(self):
        # Create publishers for sending messages
        self.msgPublisher = self.create_publisher(NavigationReport, 'msg_to_transmit_0', 10)
        self.commManUpdatedPublisher = self.create_publisher(Bool, 'comm_man_updated_0', 10)
        self.OwnNavReportPublisher = self.create_publisher(NavigationReport, 'own_nav_report_0', 10)
        self.visRecPublisher = self.create_publisher(NavigationReport, 'received_0', 10)

    def init_subscribers(self):
        # Create subscribers for receiving messages
        self.navSubscriber = self.create_subscription(Point, 'navigation_data_0', self.nav_callback, 10)
        self.reqSubscriber = self.create_subscription(String, 'msg_request_0', self.transmit_msg, 10)
        self.receiveSubscriber = self.create_subscription(NavigationReport, 'msg_received_0', self.received_callback, 10)
        self.msgIdSubscriber = self.create_subscription(Int16MultiArray, 'received_message_ids_0', self.msgId_callback, 10)

    def transmit_msg(self, msg):
        if msg.data == 'message request': 
            if not self.queueIsEmpty(self.transmit_message_queue):
                #self.get_logger().info('queue NOT empty\n')
                msg_to_transmit = self.transmit_message_queue.popleft()
                self.upload_content_ack_ids_list_to_msg(msg_to_transmit)
                self.get_logger().info(f'msg to transmit: {msg_to_transmit.message_id}\n')
                self.get_logger().info(f'ack to transmit: {msg_to_transmit.ack_ids}\n')
                self.msgPublisher.publish(msg_to_transmit)
                self.ack_message_queue.appendleft(msg_to_transmit)
                if msg_to_transmit.message_id not in self.acknowledgment_tracking:
                    self.acknowledgment_tracking[msg_to_transmit.message_id] = set()
            else: 
                response = NavigationReport()
                response.message_id = self.generate_message_id(0)
                response.ack_ids = self.convert_from_ack_ids_list_to_message(self.ack_ids_list)
                response.x = 0.0
                response.y = 0.0
                response.z = 0.0
                response.veh_class = 1
                response.battery_ok = False
                self.get_logger().info(f'msg to transmit: {response.message_id}\n')
                self.get_logger().info(f'ack to transmit: {response.ack_ids}\n')
                self.msgPublisher.publish(response)

    def queueIsEmpty(self, queue):
        if len(queue) > 0:
            return False
        else: 
            return True
        
    def nav_callback(self, msg):
        # Callback for processing incoming navigation_data messages
        message = NavigationReport()
        message.message_id = self.generate_message_id(self.message_counter)
        self.message_counter += 1
        message.ack_ids = [] 
        message.x = msg.x
        message.y = msg.y
        message.z = msg.z
        message.veh_class = self.veh_class
        message.battery_ok = self.battery_ok
        self.transmit_message_queue.append(message)
        self.OwnNavReportPublisher.publish(message)
        self.get_logger().info(f'Message added to transmit queue: {self.transmit_message_queue}\n')

    '''def received_callback(self, msg):
        # Extract agent_id and message_counter from the message ID
        agent_id, message_counter = self.extract_from_message_id(msg.message_id)

        # Handle the message as a 'ping' if the counter is 0
        if message_counter == 0:
            self.active_agents.add(agent_id)
            self.get_logger().info(f'Ping from Robot {agent_id}. Active robots: {self.active_agents}\n')

        # If the message is not already in the set, process it further
        elif msg.message_id not in self.rec_message_set:
            self.rec_message_set.add(msg.message_id)
            self.rec_message_queue.append(msg)
            self.get_logger().info(f'Message added to receive queue: {self.rec_message_queue}\n')
            self.visRecPublisher.publish(msg)

        # Update the ack_ids_queue only if the new ack_ids is different from the last one
        if not self.ack_ids_queue or msg.ack_ids != self.ack_ids_queue[0]:
            self.ack_ids_queue.appendleft(msg.ack_ids)
            # Check the ack_message_queue for messages that are acknowledged by the new ack_ids
            self.check_ack(msg.ack_ids)'''

    def received_callback(self, msg):
        # Extract agent_id and message_counter from the message ID
        agent_id, message_counter = self.extract_from_message_id(msg.message_id)

        # Handle the message as a 'ping' if the counter is 0
        if message_counter == 0:
            self.active_agents.add(agent_id)
            self.get_logger().info(f'Ping from Robot {agent_id}. Active robots: {self.active_agents}\n')

        # If the message is not already in the set, process it further
        elif msg.message_id not in self.rec_message_set:
            self.rec_message_set.add(msg.message_id)
            self.rec_message_queue.append(msg)
            self.get_logger().info(f'Message added to receive queue: {self.rec_message_queue}\n')
            self.visRecPublisher.publish(msg)

        # Update the ack_ids_queue only if the new ack_ids is different from the last one
        if not self.ack_ids_queue or msg.ack_ids != self.ack_ids_queue[0]:
            self.ack_ids_queue.appendleft(msg.ack_ids)
            for ack_id in msg.ack_ids:
                this_agent_id, _ = self.extract_from_message_id(ack_id)
                if this_agent_id == self.agent_id:
                    self.acknowledgment_tracking[ack_id].add(agent_id)
            # Check the ack_message_queue for messages that are acknowledged by the new ack_ids
            self.check_ack()
            self.cleanup_acknowledgment_tracking()

    def check_ack(self):
        temp_queue = deque()

        while self.ack_message_queue:
            message = self.ack_message_queue.popleft()
            
            acknowledged_agents = self.acknowledgment_tracking.get(message.message_id, set())
            all_agents_acknowledged = len(acknowledged_agents) == self.num_agents
            
            if all_agents_acknowledged:
                # All agents have acknowledged the message; skip retransmission
                continue
            
            # If not all agents have acknowledged, add it to the temp_queue
            temp_queue.append(message)
        
        # Now, prepend unacknowledged messages from temp_queue back to transmit_message_queue
        while temp_queue:
            unacked_msg = temp_queue.popleft()
            self.transmit_message_queue.appendleft(unacked_msg)

    def cleanup_acknowledgment_tracking(self):
        to_remove = []
        for message_id, agents in self.acknowledgment_tracking.items():
            if len(agents) == self.num_agents:
                to_remove.append(message_id)

        for message_id in to_remove:
            del self.acknowledgment_tracking[message_id]

    '''def check_ack(self, ack_ids):
        # Create a temporary queue to hold the unacknowledged messages
        temp_queue = deque()

        # Go through each message in ack_message_queue
        while self.ack_message_queue:
            message = self.ack_message_queue.popleft()
            
            # Check if the message's message_id matches any of the ack_ids
            if message.message_id in ack_ids:
                # The message has been acknowledged; continue to the next message
                continue
            
            # If the message has not been acknowledged, add it to the temp_queue
            temp_queue.append(message)
        
        # Now, prepend unacknowledged messages from temp_queue back to transmit_message_queue
        while temp_queue:
            unacked_msg = temp_queue.popleft()
            self.transmit_message_queue.appendleft(unacked_msg)'''

    def msgId_callback(self, msg):
        self.ack_ids_list = msg
        if self.ack_ids_queue:  # Check if the deque is not empty
            self.check_ack()
        else:
            # If the deque is empty, put every message from ack_message_queue back to transmit_message_queue
            while self.ack_message_queue:
                unacked_msg = self.ack_message_queue.popleft()
                self.transmit_message_queue.appendleft(unacked_msg)
        self.commManUpdatedPublisher.publish(Bool(data=True))

    def upload_content_ack_ids_list_to_msg(self, msg_to_transmit):
        msg_to_transmit.ack_ids = []
        for ack_id in self.ack_ids_list.data: 
            msg_to_transmit.ack_ids.append(ack_id)

    def convert_from_ack_ids_list_to_message(self, ack_ids_list):
        l = []
        for ack_id in ack_ids_list.data:
            l.append(ack_id)
        return l

    def generate_message_id(self, msg_counter):
        # Ensure the message ID does not exceed the allocated bits
        msg_counter %= (1 << MESSAGE_ID_BITS)
        # Combine agent ID and message ID
        message_id = (self.agent_id << MESSAGE_ID_BITS) | msg_counter
        return message_id
    
    def extract_from_message_id(self, message_id):
        # Extract message_counter from the lower MESSAGE_ID_BITS bits of message_id
        message_counter = message_id & ((1 << MESSAGE_ID_BITS) - 1)
        # Extract agent_id by right-shifting the message_id
        agent_id = message_id >> MESSAGE_ID_BITS
        return agent_id, message_counter

def main(args=None):
    rclpy.init(args=args)
    communication_manager = CommunicationManager()
    rclpy.spin(communication_manager)
    communication_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


