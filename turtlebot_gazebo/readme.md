# Overview of different folder
- /worlds : set of worlds for simulation into gazebo
- /maps: set of maps corresponding to the different simulated worlds. Allow to do navigation
- /param: set of param files especially for navigation purpose. all different configuration for the navigation are set here

# Start Simlation for mapping

```
roslaunch turtlebot_gazebo turtlebot_navigation_rooms_short_world.launch
roslaunch turtlebot_gazebo gmapping_demo.launch
roslaunch turtlebot_teleop keyboard_teleop.launch
rviz
```

# Start Simlation for navigation

```
roslaunch turtlebot_gazebo turtlebot_navigation_rooms_short_world.launch
roslaunch turtlebot_gazebo amcl_demo.launch
rviz
```

go to the maps directory and load the map

```
roscd turtlebot_gazebo/maps/
rosrun map_server map_server roomv1.yaml
```

to tune the navigation configuration go to the amcl_demo.launch file and modify the section:

```xml
<include file="$(find turtlebot_gazebo)/launch/navigation/move_base_obstacle_layer.launch.xml">
    <arg name="laser_topic" default="$(arg scan_topic)"/>
  </include>
```
use one of the following movebase configuration that respectively load configuration file from the /params directory
```
move_base_original.launch.xml
move_base_no-config.launch.xml
move_base_inflate_layer.launch.xml
move_base_obstacle_layer.launch.xml
move_base_3d_obstacle_layer.launch.xml
```
