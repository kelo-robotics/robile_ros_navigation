from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Declare the launch arguments
    config_dir = LaunchConfiguration('config_dir')
    config_dir_arg = DeclareLaunchArgument('config_dir', default_value='')
    launch_dir = LaunchConfiguration('launch_dir')
    launch_dir_arg = DeclareLaunchArgument('launch_dir', default_value='')
    
    # Create the scanners node
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                launch_dir,
                'hokuyo.launch.py'
            ])
        ]),
        launch_arguments={
            'config_dir': config_dir,
            'launch_dir': launch_dir,
            'use_sim_time': use_sim_time
        }.items()
    )

    # Create the scan merging node
    merger_node = Node(
        package='ira_laser_tools',
        executable='laserscan_multi_merger',
        name='scan_merger',
        parameters=[{
            'destination_frame': 'base_link',
            'cloud_destination_topic': '/merged_cloud',
            'scan_destination_topic': '/base_scan',
            'laserscan_topics': '/scan_1st /scan_2nd',
            'angle_min': -3.14,
            'angle_max': 3.14,
            'angle_increment': 0.0174533,
            'scan_time': 0.0333333,
            'range_min': 0.1,
            'range_max': 40.0,
        }],
        output='screen'
    )

    return LaunchDescription([
        config_dir_arg,
        launch_dir_arg,
        hokuyo_launch,
        merger_node
    ])
