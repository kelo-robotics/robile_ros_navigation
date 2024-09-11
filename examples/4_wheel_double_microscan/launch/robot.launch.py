from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import launch_ros.descriptions
import os

def generate_launch_description():
    # Configuration
    platform_name = '4_wheel_double_microscan'
    model_file = '4_wheel_double_microscan_config.urdf.xacro'
    map_name = 'empty.yaml'
    use_sim_time = 'false'
    
    map_file_path = os.path.join(get_package_share_directory('robile_ros_navigation'), 'examples', platform_name, 'map', map_name)
    model_path = os.path.join(get_package_share_directory('robile_description'), 'robots', model_file)
    package_share = FindPackageShare(package='robile_ros_navigation').find('robile_ros_navigation')
    config_dir = os.path.join(package_share, 'examples', platform_name, 'config')
    launch_dir = os.path.join(package_share, 'examples', platform_name, 'launch')
    bt_dir = os.path.join(package_share, 'examples', platform_name, 'behavior_tree')
    
    # Robot bringup
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                launch_dir,
                'bringup.launch.py'
            ])
        ]),
        launch_arguments={
            'config_dir': TextSubstitution(text=str(config_dir)),
            'launch_dir': TextSubstitution(text=str(launch_dir)),
            'model_path': TextSubstitution(text=str(model_path)),
            'use_sim_time': TextSubstitution(text=str(use_sim_time))
        }.items()
    )

    # Map server
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file_path}]
    )

    # AMCL
    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                launch_dir,
                'amcl.launch.py'
            ])
        ]),
        launch_arguments={
            'config_dir': TextSubstitution(text=str(config_dir)),
            'use_sim_time': TextSubstitution(text=str(use_sim_time))
        }.items()
    )

    # Navigation
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                launch_dir,
                'navigation.launch.py'
            ])
        ]),
        launch_arguments={
            'config_dir': TextSubstitution(text=str(config_dir)),
            'bt_dir': TextSubstitution(text=str(bt_dir)),
            'use_sim_time': TextSubstitution(text=str(use_sim_time))
        }.items()
    )
    
    # Lifecycle manager
    lifecycle_manager_node = launch_ros.actions.Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        emulate_tty=True,  
        parameters=[{'use_sim_time': False},
                    {'autostart': True},
                    {'node_names': ['amcl_node', 'map_server', 'bt_navigator', 'planner_server', 'controller_server', 'behavior_server', 'velocity_smoother']}]
    )
    
    # Visualization
    rviz2_node = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d' + os.path.join(config_dir, 'rviz', 'visualization.rviz')]
    )

    return LaunchDescription([
        bringup_launch,
        map_server_node,
        amcl_launch,
        navigation_launch,
        lifecycle_manager_node,
        rviz2_node
    ])
