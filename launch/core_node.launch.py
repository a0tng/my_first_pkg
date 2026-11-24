from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    laucnh_mode_arg = DeclareLaunchArgument(
        'launch_mode',
        default_value='simulation',
        description='launch mode type [simulation, actual]'
    )
    launch_mode = LaunchConfiguration('launch_mode')

    marker_size_arg = DeclareLaunchArgument(
        'marker_size',
        default_value='0.088',
        description='marker size double value'
    )
    marker_size = LaunchConfiguration('marker_size')

    scenario_yaml = PathJoinSubstitution([
        FindPackageShare('my_first_pkg'),
        'config',
        'scenario.yaml'
    ])

    laucnh_aruco_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('my_first_pkg'),
                'pkg_tools',
                'launch',
                'aruco.launch.py'
            ])
        ]),
        launch_arguments={
            'launch_mode': launch_mode,
            'marker_size': marker_size,
        }.items(),
    )

    launch_manipulator_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('my_first_pkg'),
                'pkg_tools',
                'launch',
                'manipulator_node.launch.py'
            ])
        ]),
    )

    core_node = Node(
            package='my_first_pkg',
            executable='core',
            name='core',
            parameters=[
                scenario_yaml,
            ]
        )

    return LaunchDescription([
        laucnh_mode_arg,
        marker_size_arg,
        laucnh_aruco_node,
        launch_manipulator_node,
        core_node,
    ])
