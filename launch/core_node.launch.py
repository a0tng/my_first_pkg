from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    launch_mode_arg = DeclareLaunchArgument(
        'launch_mode',
        default_value='actual',
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

    launch_aruco = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('my_first_pkg'),
                'pkg_tools',
                'launch',
                'aruco.launch.py'
            ])
        ]),
        launch_arguments={
            'launch_mode': 'actual',
            'marker_size': marker_size,
        }.items(),
    )

    launch_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('my_first_pkg'),
                'pkg_tools',
                'launch',
                'navigation2.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim': 'false',
            'nav2_start_rviz': 'true',

            'map_yaml_file': PathJoinSubstitution([
                FindPackageShare('my_first_pkg'),
                'pkg_tools',
                'map',
                'map.yaml'
            ]),

            'params_file': PathJoinSubstitution([
                FindPackageShare('my_first_pkg'),
                'pkg_tools',
                'param',
                'turtlebot3_hsc.yaml'
            ])
        }.items(),
    )
    
    launch_manipulator = Node(
        package='my_first_pkg',
        executable='moveit_py_demo',
        output='screen',
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
        launch_mode_arg,
        marker_size_arg,
        launch_aruco,
        launch_nav2,
        launch_manipulator,
        core_node,
    ])
