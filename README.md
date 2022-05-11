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
Please follow the instruction to install [robile_gazebo](https://github.com/kelo-robotics/robile_gazebo.git) and its dependencies
([robile_description](https://github.com/kelo-robotics/robile_description.git) and [kelo_tulip](https://github.com/kelo-robotics/kelo_tulip.git)).
Finally, install the robile_ros_navigation package using the following steps:

~~~ sh
cd ~/<CATKIN_WORKSPACE>/src
git clone https://github.com/kelo-robotics/robile_ros_navigation.git
catkin build robile_ros_navigation

source ~/catkin_ws/devel/setup.bash
~~~

## Usage

To start the example simulation launch file, execute the following command:

~~~ sh
roslaunch robile_ros_navigation simulation.launch
~~~

It will load a 3x3 ROBILE brick configuration with four active wheel that is equipped with a 2D datalogic laser scanner inside a large square room.
The modification of the robot configuration and the simulation environment is explained in the tutorial section.


For real robot example launch file, first add the sensor driver launch file to [robot.launch](examples/real_robot/launch/robot.launch?plain=1#L7).
Then start the robile_ros_navigation with:

~~~ sh
roslaunch robile_ros_navigation robot.launch
~~~

The default configuration is based on 3x3 ROBILE brick configuration with four active wheel.
Please make sure that the wheel configuration in kelo_tulip has been set properly.
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

Here are the steps to create a new simulation platform:
1. Copy the [gazebo_simulation](examples/gazebo_simulation) folder and rename it.
2. Rename the simulation.launch inside the new folder to avoid name duplication.
3. Update all file paths in simulation.launch and move_base.launch inside the new platform folder.

Here are the steps to create a new real robot platform:
1. Copy the [real_robot](examples/real_robot) folder and rename it.
2. Rename the robot.launch inside the new folder to avoid name duplication.
3. Update all file paths in robot.launch and move_base.launch inside the new platform folder.

### Step-2: Build a custom ROBILE model

Here are the steps to create a custom ROBILE model:
1. Please follow the [Adding a custom ROBILE platform](https://github.com/kelo-robotics/robile_gazebo) tutorial to create the custom ROBILE model.
2. Once the new model is created, update the "platform_name" argument in simulation.launch and make sure that the parameters in the configuration files matched the model.
3. Replace the xacro file used by "robot_description" in robot.launch or simulation.launch inside the new platform folder with the custom ROBILE xacro file created in Step-2.

### Step-3: Add sensors on the robot model

Here are the steps needed to add a new sensor to the robot model:
1. Copy the sensor xacro file to desired ros package. The default location file used in the example is in [robile_description/urdf/sensors](https://github.com/kelo-robotics/robile_description.git/urdf/sensors).
2. Add the sensor to the custom ROBILE xacro file created at Step-1. [4_wheel_lidar_config.urdf.xacro](https://github.com/kelo-robotics/robile_description.git/robots/4_wheel_lidar_config.urdf.xacro) can be used as an example.
3. Edit the [costmap_common_params.yaml](config/costmap_common_params.yaml) so it uses the correct sensor configuration.

### Step-4: Update the robot footprint

Update the footprint of the robot in move_base_params.yaml inside the config folder so it fits the custom ROBILE platform.
As a convention, the front side of the robot is the positive x-axis and the left side of the robot is the positive y-axis.
The footprint consists of a series of points (x, y) which are ordered sequentially.

## TUTORIAL: Adding a new map

### Step-1: Create a 2D map of the new environment

Please follow the tutorial for [creating a 2D map using gmapping from a recorded bagfile](http://wiki.ros.org/slam_gmapping/Tutorials/MappingFromLoggedData).
Once the map is created, copy the yaml and pgm file to the map folder of the project.

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

For real robot simply update the "map" argument in robot.launch to use the new map.
Meanwhile for simulation, update "world_model_name" argument in simulation.launch.



