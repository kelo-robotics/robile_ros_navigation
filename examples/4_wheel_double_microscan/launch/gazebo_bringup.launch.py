import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    use_sim_time = True

    gazebo_pkg = get_package_share_directory("robile_gazebo")

    world_file = os.path.join(
        get_package_share_directory("robile_ros_navigation"),
        "examples",
        "4_wheel_double_microscan",
        "map",
        "office.world",
    )

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, "launch", "simulation.launch.py")
        ),
        launch_arguments={
            "config": "4_wheel_double_microscan_config",
            "world": world_file,
            "use_sim_time": "true",
            "init_pos_x": "2.0",
        }.items(),
    )

    scanner_1_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/scanner_1/scan_filtered@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan"
        ],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    scanner_2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/scanner_2/scan_filtered@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan"
        ],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    merger_node = Node(
        package="ira_laser_tools",
        executable="laserscan_multi_merger",
        name="scan_merger",
        parameters=[
            {
                "destination_frame": "base_link",
                "cloud_destination_topic": "/merged_cloud",
                "scan_destination_topic": "/base_scan",
                "laserscan_topics": "/scanner_1/scan_filtered /scanner_2/scan_filtered",
                "angle_min": -3.14,
                "angle_max": 3.14,
                "angle_increment": 0.0174533,
                "scan_time": 0.0333333,
                "range_min": 0.1,
                "range_max": 40.0,
            }
        ],
        output="screen",
    )
    return LaunchDescription(
        [
            sim_launch,
            scanner_1_bridge,
            scanner_2_bridge,
            merger_node,
        ]
    )
