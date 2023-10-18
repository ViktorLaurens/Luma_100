import os 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('comm_man'), 
        'config',
        'params.yaml'
    )

    my_node = Node(
        package='comm_man',
        executable='man_node',
        name='man_node',
        parameters= [config]
    )

    ld.add_action(my_node)
    return ld