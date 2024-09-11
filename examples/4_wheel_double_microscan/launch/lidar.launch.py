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

    # Create the scanners node
    scanner1_node = Node(
        package='sick_safetyscanners2',
        executable='sick_safetyscanners2_node',
        name='scanner_1',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                config_dir,
                'scanner.yaml'
            ])
        ],
        remappings=[
            ('scan', '/scanner_1/scan_filtered'),
        ]
    )

    scanner2_node = Node(
        package='sick_safetyscanners2',
        executable='sick_safetyscanners2_node',
        name='scanner_2',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                config_dir,
                'scanner.yaml'
            ])
        ],
        remappings=[
            ('scan', '/scanner_2/scan_filtered'),
        ]
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
            'laserscan_topics': '/scanner_1/scan_filtered /scanner_2/scan_filtered',
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
        scanner1_node,
        scanner2_node,
        merger_node
    ])
