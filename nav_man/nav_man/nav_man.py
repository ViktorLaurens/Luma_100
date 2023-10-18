import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import random

class NavigationManager(Node):

    def __init__(self):
        super().__init__('navigation_manager')
        # Initialize parameters, publishers, subscribers, and other variables
        self.load_parameters()
        self.load_publishers()
        self.load_subscribers()
        # Set up timer for periodic tasks
        self.timer_ = self.create_timer(2.0, self.publish_data)

    def load_parameters(self):
        # Declare parameters
        #self.total_number_of_msgs = 0
        pass

    def load_publishers(self):
        # Create publishers for sending messages
        self.navPublisher = self.create_publisher(Point, 'navigation_data_0', 10)

    def load_subscribers(self):
        # Create subscribers for receiving messages
        pass

    def publish_data(self):
        # Publish navigation report 
        #if self.total_number_of_msgs < 5:
            #self.total_number_of_msgs += 1
        data = Point()
        #data.id = self.generate_message_id()
        data.x = round(random.uniform(-500.0, 500.0), 1)
        data.y = round(random.uniform(-500.0, 500.0), 1)
        data.z = round(random.uniform(-500.0, 500.0), 1)
        #data.veh_class = 1
        #data.battery_ok = True
        self.navPublisher.publish(data)
        self.get_logger().info('Publishing state...')

def main(args=None):
    rclpy.init(args=args)
    nav_man_node = NavigationManager()
    rclpy.spin(nav_man_node)
    nav_man_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

