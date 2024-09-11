from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml
from launch import LaunchContext
import os

def generate_launch_description():
    # Declare the launch arguments
    config_dir = LaunchConfiguration('config_dir')
    bt_dir = LaunchConfiguration('bt_dir')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    config_dir_arg = DeclareLaunchArgument('config_dir', default_value='')
    bt_dir_arg = DeclareLaunchArgument('bt_dir', default_value='')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    
    #bt_navigator_yaml = PathJoinSubstitution([ config_dir, 'behavior_tree_params.yaml' ])
    # Create the navigator node
    bt_navigator_yaml_raw = PathJoinSubstitution([ config_dir, 'behavior_tree_params.yaml' ])

    context = LaunchContext()
    param_substitutions = {
        'default_nav_to_pose_bt_xml': PathJoinSubstitution([bt_dir, 'nav_to_pose_bt.xml']),
        'default_nav_through_poses_bt_xml': PathJoinSubstitution([bt_dir, 'nav_through_poses_bt.xml'])
    }

    configured_params = RewrittenYaml(
        source_file=bt_navigator_yaml_raw,
        root_key='',
        param_rewrites=param_substitutions
    )

    bt_navigator_yaml = configured_params.perform(context)
    
    navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            bt_navigator_yaml,
            {'use_sim_time': use_sim_time}
        ]
    )

    # Create the planner node
    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            #os.path.join(config_dir, 'planner_server.yaml'),
            PathJoinSubstitution([
                config_dir,
                'planner_server.yaml'
            ]),
            {'use_sim_time': use_sim_time}
        ]
    )

    # Create the controller node
    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[
            #os.path.join(config_dir, 'controller_server.yaml'),
            PathJoinSubstitution([
                config_dir,
                'controller_server.yaml'
            ]),
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('cmd_vel', 'cmd_vel'),
            ('odom', 'odom'),
        ]
    )

    # Create the recovery behavior node
    behavior_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[
            #os.path.join(config_dir, 'behavior_server.yaml'),
            PathJoinSubstitution([
                config_dir,
                'behavior_server.yaml'
            ]),
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('cmd_vel', 'cmd_vel'),
            ('odom', 'odom'),
        ]
    )
    
    # Create the velocity smoother node
    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[
            #os.path.join(config_dir, 'velocity_smoother.yaml'),
            PathJoinSubstitution([
                config_dir,
                'velocity_smoother.yaml'
            ]),
            {'use_sim_time': use_sim_time}
        ]
    )

    return LaunchDescription([
        config_dir_arg,
        bt_dir_arg,
        use_sim_time_arg,
        navigator_node,
        planner_node,
        controller_node,
        behavior_node,
        velocity_smoother_node
    ])
