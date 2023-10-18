import rclpy
from rclpy.node import Node

class MyNode(Node):
    
    def __init__(self):
        super().__init__('py_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('baud_rate', rclpy.Parameter.Type.INTEGER),
                ('agent_id', rclpy.Parameter.Type.INTEGER),
                ('time_slot_duration', rclpy.Parameter.Type.DOUBLE),
                ('num_agents', rclpy.Parameter.Type.INTEGER),
                ('message_written_flag', rclpy.Parameter.Type.BOOL),
                ('header_received', rclpy.Parameter.Type.BOOL),
                ('message_length', rclpy.Parameter.Type.INTEGER)
            ]
        )

def main(args=None):
    rclpy.init(args=args)
    node=MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()