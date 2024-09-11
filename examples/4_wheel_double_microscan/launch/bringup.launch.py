from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import FindExecutable, PathJoinSubstitution, Command, TextSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import launch_ros.descriptions
import os

def generate_launch_description():
    # Declare the launch arguments
    robot_name = os.environ.get('ROBOT_NAME', 'example') 
    driver_config_file = os.path.join(get_package_share_directory("kelo_tulip"), "config", robot_name + ".yaml")
    config_dir = LaunchConfiguration('config_dir')
    launch_dir = LaunchConfiguration('launch_dir')
    model_path = LaunchConfiguration('model_path')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    config_dir_arg = DeclareLaunchArgument('config_dir', default_value='')
    launch_dir_arg = DeclareLaunchArgument('launch_dir', default_value='')
    model_path_arg = DeclareLaunchArgument('model_path', default_value='')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='')
    
    # Kelo drive driver
    kelo_tulip_node = Node(
        package='kelo_tulip',
        executable='platform_driver',
        name='platform_driver',
        output='screen',
        parameters=[driver_config_file],
        remappings=[
            ('cmd_vel', 'cmd_vel_smoothed'),
            ('odom', 'odom'),
        ]
    )

    # Joypad
    joypad_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy_linux_node',
        output='screen'
    )

    # Lidar driver
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                launch_dir,
                'lidar.launch.py'
            ])
        ]),
        launch_arguments={
            'config_dir': config_dir,
            'use_sim_time': use_sim_time
        }.items()
    )

    # Robot state publisher
    robot_description = Command([
        FindExecutable(name='xacro'),
        ' ',
        model_path
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': launch_ros.descriptions.ParameterValue(robot_description, value_type=str),
            'publish_frequency': 50.0,
        }]
    )

    # Joint state publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_gui': 'false',
        }]
    )

    return LaunchDescription([
        config_dir_arg,
        launch_dir_arg,
        model_path_arg,
        use_sim_time_arg,
        kelo_tulip_node,
        joypad_node,
        lidar_launch,
        robot_state_publisher_node,
        joint_state_publisher_node,
    ])
