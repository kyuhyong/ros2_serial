import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('ros2_serial'),
        'config',
        'sonar.yaml'
        )
    return LaunchDescription([
        Node(
            package = 'ros2_serial',
            executable = 'ros2_serial_node',
            output = 'screen',
            parameters = [config]
        )
    ])