from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
#import os

def generate_launch_description():
    # Declare the launch arguments
    config_dir = LaunchConfiguration('config_dir')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    config_dir_arg = DeclareLaunchArgument('config_dir', default_value='')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='')
    map_arg = DeclareLaunchArgument('map_topic', default_value='map')
    
    return LaunchDescription([
        map_arg,
        config_dir_arg,
        use_sim_time_arg,
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    config_dir,
                    'amcl.yaml'
                ])
            ],
        )
    ])
