# robile_ros_navigation

ROS 2 Navigation for KELO ROBILE (tested on ROS Humble). 
For ROS 1 version please read the README.md file in the master branch.

## Installation
Here are the required packages for robile_ros_navigation package:
- ROS 2 Navigation
- robile_gazebo
- kelo_tulip
- sick_microscanner2
- ira_laser_tools
- joy

First install the robile packages and ira_laser_tools from github by replacing <WORKSPACE_DIR> with the correct ros workspace path:

~~~ sh
cd <WORKSPACE_DIR>/src
git clone -b humble https://github.com/nakai-omer/ira_laser_tools.git
git clone https://github.com/kelo-robotics/robile_description.git
git clone -b ros2-develop https://github.com/kelo-robotics/kelo_tulip.git
git clone -b ros2-develop https://github.com/kelo-robotics/robile_gazebo.git
git clone -b ros2-develop https://github.com/kelo-robotics/robile_ros_navigation.git
~~~

Next install the remaining dependencies using rosdep:

~~~ sh
rosdep install --from-paths src -y --ignore-src
~~~

Finally compile the packages inside the ros workspace with the following commands by replacing <WORKSPACE_DIR> with the correct ros workspace path: 

~~~ sh
cd <WORKSPACE_DIR>
colcon build
source ~/<WORKSPACE>/install/local_setup.bash
~~~

## Usage

To start the example simulation launch file, execute the following command:

~~~ sh
ros2 launch robile_ros_navigation simulation.launch.py
~~~

It will load a 2X3 ROBILE brick configuration with four active wheel with two sick microscanners inside a large square room.
The modification of the robot configuration and the simulation environment is explained in the tutorial section.


For real robot example launch file execute the following command:

~~~ sh
ros2 launch robile_ros_navigation robot.launch.py
~~~

The default configuration is based on 2X3 ROBILE brick configuration with four active wheel with two sick microscanners.
For more information on the configuration file, please read the documentation of [kelo_tulip](https://github.com/kelo-robotics/kelo_tulip).

When using kelo_tulip as the driver it is also possible to drive or override the robot using a joystick.
The control is as follows:
- RB: safety switch. This button needs to be pressed so the joystick command can be active.
- Left analog stick: controls linear velocity of the robot.
- Right analog stick: controls rotation velocity of the robot.

Note: the robot will immediately use the velocity command from ROS Navigation once the RB button was released.
Please make sure that the navigation command has been canceled.


## TUTORIAL: Create a custom ROBILE platform

In this section we describe the procedure to create a custom ROBILE platform configurations.

### Step-1: Create a new platform folder

Here are the steps to create a new real robot platform:
1. Copy the [4_wheel_double_microscan](examples/4_wheel_double_microscan) folder and rename it.
2. Rename the robot.launch.py inside the new folder to avoid name duplication.
3. Change the platform name in [robot.launch.py](examples/4_wheel_double_microscan/launch/robot.launch.py) to the new platform name.
4. Add the new platform to the [CMakeLists.txt](CMakeLists.txt).

~~~ sh
install(DIRECTORY
  examples/<ROBOT_NAME>/launch
  examples/<ROBOT_NAME>/config
  examples/<ROBOT_NAME>/map
  examples/<ROBOT_NAME>/behavior_tree
  DESTINATION share/${PROJECT_NAME}/examples/<ROBOT_NAME>
)
~~~

5. When needed, replace the [lidar.launch.py](examples/4_wheel_double_microscan/launch/lidar.launch.py) with the correct lidar driver.

### Step-2: Build a custom ROBILE model

Here are the steps to create a custom ROBILE model:
1. Please follow the [Building a custom ROBILE platform configuration](https://github.com/kelo-robotics/robile_description) tutorial in robile_description package.
2. For simulation please follow the [Adding a custom ROBILE platform](https://github.com/kelo-robotics/robile_gazebo) tutorial in robile_gazebo package.
3. Once the new model is created save the xacro file inside [robots](https://github.com/kelo-robotics/robile_description.git/robots) folder.
4. Update the "model_file" argument in the new [robot.launch.py](examples/4_wheel_double_microscan/launch/robot.launch.py) file in robile_ros_navigation with the correct file name.

### Step-3: Add sensors on the robot model

Here are the steps needed to add a new sensor to the robot model:
1. Copy the sensor file to the desired directory. The default location file used in the example is in [robile_description/urdf/sensors](https://github.com/kelo-robotics/robile_description.git/urdf/sensors).
2. Add the sensor to the custom ROBILE xacro file created at Step-1. [4_wheel_double_microscan_config.urdf.xacro](https://github.com/kelo-robotics/robile_description.git/robots/4_wheel_double_microscan_config.urdf.xacro) can be used as an example.
3. Create a new launch file for the lidars and replace the default lidar launch file inside [bringup.launch.py]((examples/4_wheel_double_microscan/launch/bringup.launch.py).

### Step-4: Update the robot footprint

Update the footprint of the robot in [planner_server.yaml](examples/4_wheel_double_microscan/config/planner_server.yaml) and [controller_server.yaml](examples/4_wheel_double_microscan/config/controller_server.yaml) so it fits the custom ROBILE platform.
As a convention, the front side of the robot is the positive x-axis and the left side of the robot is the positive y-axis.
The footprint consists of a series of points (x, y) which are ordered sequentially.

## TUTORIAL: Adding a new map

### Step-1: Create a 2D map of the new environment

Please follow the tutorial for [creating a 2D map using gmapping from a recorded bagfile](http://wiki.ros.org/slam_gmapping/Tutorials/MappingFromLoggedData).
Once the map is created, copy the yaml and pgm file to the [map](examples/4_wheel_double_microscan/map) folder.

### Step-2: Create xacro and stl file of the map (Simulation only)

A simple 3D model (stl) can be created by extruding the 2D image of the map. The procedure is as follows:
1. Open inkscape and import the pgm file of the 2D map.
2. Select the image and then click path -> trace bitmap and then press ‘OK’.
3. Now the lines of the wall should be created. Remove the 2D map so only the wall lines are left.
4. Save as .svg file.
5. Open Blender and then remove the box in the center by clicking it and then press ‘x’ and select ‘delete’.
6. Import the .svg file and then select on the scene box ‘Curve 1’.
7. Press alt+c and select ‘Mesh from Curve/Meta/Surf/Text’.
8. Move the map to the center by using the arrows.
9. Press ctrl+a and select ‘Location’.
10. Press ‘tab’ to enter the edit mode.
11. Press ‘a’ to select the map and then ‘e’ to extrude.
12. Move the mouse to change the width of the extrusion and then left click.
13. Save the extruded map as .stl file and copy it to desired simulation platform map folder.

Then copy the empty.xacro file inside the new platform folder and rename it with the new map name and change the model to the new stl file.

### Step-3: Change the loaded map in the launch file

Change the map name in [robot.launch.py](examples/4_wheel_double_microscan/launch/robot.launch.py) to the new map name.


