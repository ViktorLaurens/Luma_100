#Thinkpad

import rclpy
from rclpy.node import Node
from custom_interfaces.msg import NavigationReport
from collections import deque 
import matplotlib.pyplot as plt
from datetime import datetime

# Constants for the number of bits allocated for agent ID and message ID
AGENT_ID_BITS = 3  # Adjust based on the number of agents (e.g., 2, 3, 4, ...)
MESSAGE_ID_BITS = 9  # Adjust based on the expected number of messages (e.g., 256, 512, ...)

class Visualization(Node):
    def __init__(self, ):
        super().__init__('visualization')
        # Initialize parameters, publishers, subscribers, and other variables
        self.init_parameters()
        self.init_subscribers()
        self.start_time = datetime.now()
        # Initialize the plot
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlabel("Time")
        self.ax.set_ylabel("Robot ID")
        self.ax.set_title("Messages Received Over Time")
        plt.ion()
        plt.show()

    def init_parameters(self):
        # Initialize message queue for the sent messages
        #self.send_message_queue = deque()
        # Initialize message queue for the received messages  
        #self.rec_message_queue = deque()
        self.sent_ids = []
        self.received_ids = {}
        self.x_data = []  # To store the current_time values for the last 10 messages
        self.window_size = 10  # Number of messages in the moving window

    def init_subscribers(self):
        # Create subscribers for receiving messages
        self.OwnNavReportSubscriber = self.create_subscription(NavigationReport, 'own_nav_report_0', self.nav_callback, 10)
        self.visRecSubscriber = self.create_subscription(NavigationReport, 'received_0', self.received_callback, 10)

    def nav_callback(self, msg):
        # Extract the data for plotting
        agent_id, message_counter = self.extract_from_message_id(msg.message_id)
        self.sent_ids.append(message_counter)
        
        # Formatting the log message
        log_message = '---'.join(map(str, self.sent_ids))
        self.get_logger().info(f'Robot {agent_id} sent: {log_message}\n')

        # Get current time difference from the start time in seconds
        current_time = (datetime.now() - self.start_time).total_seconds()

        # Plotting
        self.ax.scatter(current_time, agent_id, color='red')  # Use a different color to distinguish sent messages
        self.ax.text(current_time, agent_id, str(message_counter))

        # Adjust moving window
        self.x_data.append(current_time)
        if len(self.x_data) > self.window_size:
            self.x_data.pop(0)
        self.ax.set_xlim(self.x_data[0], current_time)
        
        # Adjust Y-axis
        highest_agent_id = max(max(self.received_ids.keys(), default=-1), agent_id)
        self.ax.set_ylim(-1, highest_agent_id + 1)
        self.ax.set_yticks(range(-1, highest_agent_id + 2))  # Setting the Y-ticks to integers
        
        plt.draw()
        plt.pause(0.001)  # pause for a short duration to allow the plot to update

    def received_callback(self, msg):
        agent_id, message_counter = self.extract_from_message_id(msg.message_id)

        # Check if agent_id is already a key in the dictionary
        if agent_id not in self.received_ids:
            self.received_ids[agent_id] = []
        self.received_ids[agent_id].append(message_counter)
        
        # Formatting the log message
        log_message = '---'.join(map(str, self.received_ids[agent_id]))
        self.get_logger().info(f'Received from robot {agent_id}: {log_message}\n')

        # Get current time difference from the start time in seconds
        current_time = (datetime.now() - self.start_time).total_seconds()
        # Plotting
        self.ax.scatter(current_time, agent_id, color='blue')
        self.ax.text(current_time, agent_id, str(message_counter))
        
        # Adjust moving window
        self.x_data.append(current_time)
        if len(self.x_data) > self.window_size:
            self.x_data.pop(0)
        self.ax.set_xlim(self.x_data[0], current_time)

        # Adjust Y-axis
        highest_agent_id = max(max(self.received_ids.keys(), default=-1), agent_id)
        self.ax.set_ylim(-1, highest_agent_id + 1)
        self.ax.set_yticks(range(-1, highest_agent_id + 2))  # Setting the Y-ticks to integers
        
        plt.draw()
        plt.pause(0.001)  # pause for a short duration to allow the plot to update

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
