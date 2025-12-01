import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros.descriptions


def generate_launch_description():

    platform_name = "4_wheel_double_uam"
    map_name = "office.yaml"
    use_sim_time = "true"

    package_share = FindPackageShare(package="robile_ros_navigation").find(
        "robile_ros_navigation"
    )
    map_file_path = os.path.join(
        package_share, "examples", platform_name, "map", map_name
    )
    config_dir = os.path.join(package_share, "examples", platform_name, "config")
    launch_dir = os.path.join(package_share, "examples", platform_name, "launch")
    bt_dir = os.path.join(package_share, "examples", platform_name, "behavior_tree")

    # Robot simulation bringup
    sim_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "gazebo_bringup.launch.py")
        ),
    )

    # Map server
    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        output="screen",
        parameters=[{"yaml_filename": map_file_path}],
    )

    # AMCL
    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([launch_dir, "amcl.launch.py"])]
        ),
        launch_arguments={
            "config_dir": config_dir,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # Navigation
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([launch_dir, "navigation.launch.py"])]
        ),
        launch_arguments={
            "config_dir": config_dir,
            "bt_dir": bt_dir,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # Lifecycle manager
    lifecycle_manager_node = launch_ros.actions.Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"use_sim_time": True},
            {"autostart": True},
            {
                "node_names": [
                    "amcl_node",
                    "map_server",
                    "bt_navigator",
                    "planner_server",
                    "controller_server",
                    "behavior_server",
                    "velocity_smoother",
                ]
            },
        ],
    )

    # Visualization
    rviz2_node = Node(
        package="rviz2",
        namespace="",
        executable="rviz2",
        name="rviz2",
        arguments=["-d" + os.path.join(config_dir, "rviz", "visualization.rviz")],
    )

    return LaunchDescription(
        [
            sim_bringup_launch,
            map_server_node,
            amcl_launch,
            navigation_launch,
            lifecycle_manager_node,
            rviz2_node,
        ]
    )
