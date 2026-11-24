import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import AppendEnvironmentVariable


def generate_launch_description():

    ld = LaunchDescription()
    model_path = PathJoinSubstitution([
        FindPackageShare('my_first_pkg'),
        'pkg_tools',
        'models'
    ])

    ld.add_action(AppendEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=model_path
    ))
    launch_dir = os.path.join(
        get_package_share_directory(
            'turtlebot3_manipulation_moveit_config'), 'launch')
    bringup_launch_dir = os.path.join(
        get_package_share_directory(
            'turtlebot3_manipulation_gazebo'), 'launch')

    # RViz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_dir, '/moveit_rviz.launch.py'])
    )
    ld.add_action(rviz_launch)

    # move_group
    move_group_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_dir, '/move_group.launch.py']),
            launch_arguments={
                'use_sim': 'true',
            }.items(),
        )
    ld.add_action(move_group_launch)

    # gazebo_control with robot_state_publisher
    rviz_arg = DeclareLaunchArgument(
        'start_rviz',
        default_value='false',
        description='Whether execute rviz2')
    ld.add_action(rviz_arg)

    challenge_world_path = PathJoinSubstitution(
        [
            FindPackageShare('my_first_pkg'),
            'pkg_tools',
            'worlds',
            'challenge.world'
        ]
    )

    gazebo_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_launch_dir, '/gazebo.launch.py']),
        launch_arguments={
            'world': challenge_world_path,
            'start_rviz': 'false',
            'x_pose': '0.0',
            'y_pose': '0.0',
            'z_pose': '0.0',
            'roll': '0.0',
            'pitch': '0.0',
            'yaw': '0.0',
        }.items(),
        )
    ld.add_action(gazebo_control_launch)

    return ld
