from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='turtlesim', executable='turtlesim_node'),
        Node(package='my_first_pkg',
             executable='square_driver',
             parameters=[{'speed': 1.0, 'turn': 1.0}]),
    ])
