from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    laucnh_mode_arg = DeclareLaunchArgument(
        'launch_mode',
        default_value='simulation',
        description='launch mode type [simulation, actual]')
    launch_mode = LaunchConfiguration('launch_mode')

    marker_size_arg = DeclareLaunchArgument(
        'marker_size',
        default_value='0.088',
        description='marker size double value')
    marker_size = LaunchConfiguration('marker_size')

    tracker_node = Node(
        package='my_first_pkg',
        executable='aruco_tracker',
        name='aruco_tracker',
        output='screen',
        parameters=[{
            'launch_mode': launch_mode,
            'marker_size': marker_size,
        }],
    )

    parking_node = Node(
        package='my_first_pkg',
        executable='aruco_parking',
        name='aruco_parking',
        output='screen',
    )

    return LaunchDescription([
        laucnh_mode_arg,
        marker_size_arg,
        tracker_node,
        parking_node,
    ])
