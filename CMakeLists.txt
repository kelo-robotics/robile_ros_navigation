cmake_minimum_required(VERSION 3.5.1)
project(robile_ros_navigation)


# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(kelo_tulip REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(
    include
    ${kelo_tulip_INCLUDE_DIRS}
)

# Install launch files
install(DIRECTORY
  examples/4_wheel_double_microscan/launch
  examples/4_wheel_double_microscan/config
  examples/4_wheel_double_microscan/map
  examples/4_wheel_double_microscan/behavior_tree
  DESTINATION share/${PROJECT_NAME}/examples/4_wheel_double_microscan
)

#install(DIRECTORY
  #examples/<ROBOT_NAME>/launch
  #examples/<ROBOT_NAME>/config
  #examples/<ROBOT_NAME>/map
  #examples/<ROBOT_NAME>/behavior_tree
  #DESTINATION share/${PROJECT_NAME}/examples/<ROBOT_NAME>
#)

ament_package()
