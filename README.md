# robile_ros_navigation

ROS Navigation for KELO ROBILE robots

## Installation
Here are the required packages for robile_ros_navigation package:
- ros-navigation
- robile_gazebo
- joy (optional)

First install ros-navigation and joy using the following command:

~~~ sh
sudo apt install ros-$ROS_DISTRO-navigation ros-$ROS_DISTRO-joy
~~~

Then install robile_gazebo package from KELO-robotics github.
Please follow the instruction to install [robile_gazebo](https://github.com/kelo-robotics/robile_gazebo.git) and its dependencies.
Afterwards, change the branch of [robile_description](https://github.com/kelo-robotics/robile_description.git) and [robile_gazebo](https://github.com/kelo-robotics/robile_gazebo.git) to "move_base".
Finally, install the robile_ros_navigation package using the following steps:

~~~ sh
cd ~/<CATKIN_WORKSPACE>/src/robile_description
git checkout move_base
catkin build robile_description

cd ~/<CATKIN_WORKSPACE>/src/robile_gazebo
git checkout move_base
catkin build robile_gazebo

cd ~/<CATKIN_WORKSPACE>/src
git clone 
git clone https://github.com/kelo-robotics/robile_ros_navigation.git

catkin build robile_navigation
source ~/catkin_ws/devel/setup.bash
~~~

## Usage

To start the example simulation launch file, execute the following command:

~~~ sh
roslaunch robile_ros_navigation simulation.launch
~~~

It will load a 3x3 ROBILE brick configuration with four active wheel and a SICK laser scanner inside a square room.
The modification of the robot configuration and the simulation environment is explained in the tutorial section.

To start the robile_ros_navigation with kelo_tulip driver, launch:

~~~ sh
roslaunch robile_ros_navigation robot.launch
~~~

The default configuration is based on 3x3 ROBILE brick configuration with four active wheel.
Please make sure that the wheel configuration in kelo_tulip has been set properly.
For more information on the configuration file, please read the documentation of [kelo_tulip](https://github.com/kelo-robotics/kelo_tulip).

For a real robot it is also possible to drive or override the robot using a joystick.
The control is as follows:
- RB: safety switch. This button needs to be pressed so the joystick command can be active.
- left analog stick: controls linear velocity of the robot.
- right analog stick: controls rotation velocity of the robot.

Note: the robot will immediately use the velocity command from ROS Navigation once the RB button was released.
Please make sure that the navigation command has been canceled.

## TUTORIAL: Create a custom ROBILE platform

In this section we describe the procedure to create a custom ROBILE platform configurations.

### Step-1: Build a custom ROBILE platform configuration

Please follow the [Adding a custom ROBILE platform](https://github.com/kelo-robotics/robile_gazebo) tutorial.

### Step-2: Add sensors on the robot model

Here are the steps needed to add a new sensor to the robot model:
1. copy the sensor xacro file to desired ros package. The LMS1xx xacro file used on the default robot can be found in robile_description/urdf/sensors.
2. add the sensor to the custom ROBILE xacro file created at step-1. The xacro file of the [3x3 ROBILE with a SICK scanner](https://github.com/kelo-robotics/robile_description.git/robots/4_wheel_lidar_config.urdf.xacro) can be used as an example.
3. replace the xacro file used by "robot_description" in [robot.launch](launch/robot.launch) and [simulation.launch](launch/simulation.launch) with the custom ROBILE xacro file.
4. edit the [costmap_common_params.yaml](config/costmap_common_params.yaml) so it uses the correct sensor configuration.

For the real robot, make sure to install the sensor driver and include the driver launch file to [robot.launch](launch/robot.launch).

For simulation, edit robile_ros_navigation/launch/simulation.launch so it uses the new launch file for the custom ROBILE.

### Step-3: Update the robot footprint

Update the footprint for move_base in [move_base_params.yaml](config/move_base_params.yaml) to fit the custom ROBILE platform.
As a convention, the front side of the robot is the positive x-axis and the left side of the robot is the positive y-axis.
The footprint consists of a series of points (x, y) which are ordered sequentially. Here is an example of the footprint:

~~~
footprint: [[0.3345, 0.223],
            [0.3345, 0.1],
            [0.55, 0.1],
            [0.55, -0.1],
            [0.3345, -0.1],
            [0.3345, -0.223],
            [-0.3345, -0.223],
            [-0.3345, 0.223]]
~~~

## TUTORIAL: Adding a new map

### Step-1: Create a 2D map of the new environment

Please follow the tutorial for [creating a 2D map using gmapping from a recorded bagfile](http://wiki.ros.org/slam_gmapping/Tutorials/MappingFromLoggedData).
Once the map is created, copy the yaml and pgm file into [map](map/) folder.

### Step-2: Create world and stl file of the map (Simulation only)

The stl file can be created by extruding the 2D image of the map. The procedure is as follows:
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
13. Save the extruded map as .stl file and copy it to the [map](map/) folder.

Then copy the [empty.world](map/empty.world) file and rename it with the new map name and change the model to the new stl file.

### Step-3: Change the loaded map in the launch file

For real robot, simply update the "_map" argument in robot.launch.
Meanwhile for simulation, update "_map", "_gazebo_world_path", "_gazebo_world" arguments in simulation.launch.



