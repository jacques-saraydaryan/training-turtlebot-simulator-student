# ROS Navigation Tutorial

## 0. Introduction

The following document presents an incremental overview of the different ROS configuration files and parameters used in Ros Navigation Package.
Before starting have a look to the following resources:
 - ROS [move_base](http://wiki.ros.org/move_base) package
 - ROS [costmap_2d](http://wiki.ros.org/costmap_2d) package
 - ROS wiki page of [Basic Navigation Tuning Guide](http://wiki.ros.org/navigation/Tutorials/Navigation%20Tuning%20Guide)
 - The excellent ROS navigation tuning guide providing by Kaiyu Zheng [here](http://kaiyuzheng.me/documents/navguide.pdf)

<img src="http://wiki.ros.org/navigation/Tutorials/RobotSetup?action=AttachFile&do=get&target=overview_tf.png" alt="ROS1 Navigation Stack" width="600"/>

## 1. Start Simulation and Mapping
### 1.1. Start simulator for mapping
- Follow the instructions provided in the section **Start Simlation for mapping** of the [readme.md](./readme.md).
### 1.2. Configure Rviz 
- Configure Rviz to see the following:
  - Map generated (topic `/map`)
  - Laser information (topic `/laserscan`)
    - `Style`: Boxes
    - `Size`: 0.07
    - `Color Transformer`: FlatColor
    - `Color`: 255; 0; 0
  - Kinect "Laser" information (topic `/kinect_scan`)
    - `Style`: Boxes
    - `Size`: 0.1
    - `Color Transformer`: FlatColor
    - `Color`: 252; 175; 62
### 1.3 Map the environment
  - Begin to map the environment with the teleop
  - What happened when the robot move ? Why ?
  - Remember how the map is built (behaviour of affordance map)
  - What happened when your robot tries to map a long corridor ? Explain

### 1.4 Map files
  - After mapping some rooms, save your map with the following command:

  ```
    rosrun map_server map_saver -f myMap

  ```
  - 2 files are generated:
    - `myMap.yaml`
    - `myMap.pgm`
  
  - Open the myMap.yaml and explain each lines

### 1.5 Change observation source
  - Stop your gmapping node
  - Uncomment the following line in the `gmapping_demo.launch` file
  ```xml
    <arg name="scan_topic"  default="kinect_scan" />
  ```
  - This line changes the observation source from laser to kinect "laser"
  - Start again your gmapping node and try to map the environment
  - What do you observe ? Why ?

### 1.6 ref
  - [http://wiki.ros.org/map_server](http://wiki.ros.org/map_server)


## 2. Start simulation env.


- Follow the instructions provided in the section **Start Simlation for navigation** of the [readme.md](./readme.md).
 Use the **move_base_no-config.launch.xml** in the **amcl_demo.launch** to start the basic configuration.

- Check that in the env. is correctly load into gazebo (env. plus robot).

- Teleoperate the robot into the simulation.

```
roslaunch turtlebot_teleop keyboard_teleop.launch
```

- Try to ask a navigation throught riz
    1. Relocate the robot using the **2D Pose Estimate** tool
    1. Ask goal with the **2D Nav Goal** tool

- What happened when the robot tries to cross a door ? Why ?


## 3. Inflate layer

### 3.1 Configuration

1. Copy all configuration files from the param/no-config folder to the param/inflate_layer folder.
2. Change the launch file as follow to use your new config.
 in the amcl_demo.launch file :

```xml
<include file="$(find turtlebot_gazebo)/launch/navigation/move_base_inflate_layer.launch.xml">
    <arg name="laser_topic" default="$(arg scan_topic)"/>
  </include>
```


### 3.2 Add an inflate layer
In the configuration file **costmap_common_params.yaml**, define an inflate layer (follow documentation in [costmap_2d]()) as follow:
```yaml
robot_radius: 0.20  
map_type: voxel

static_layer:
  enabled:              true

#New section
inflation_layer:
  enabled:              true
  cost_scaling_factor:  5  
  inflation_radius:     0.5


```

Find the definition of each parameter:
 - Global parameters:
    - robot_radius : radius of the robot use to compute cost into inflate layer (refer to inscribed_radius)
    - map_type: type of map used to compute path ([voxel](http://wiki.ros.org/voxel_grid))
 - [Static layer](http://wiki.ros.org/costmap_2d/hydro/staticmap)
 - [Inflation layer](http://wiki.ros.org/costmap_2d/hydro/inflation)


 Go to the **global_costmap_params.yaml** configuration file and add the inflate layer:

```yaml
 global_costmap:
   lobal_frame: /map
   robot_base_frame: /base_footprint
   update_frequency: 2.5
   publish_frequency: 2.0
   transform_tolerance: 0.5

   #Updated section
   plugins:
     - {name: static_layer,            type: "costmap_2d::StaticLayer"}
     - {name: inflation_layer,         type: "costmap_2d::InflationLayer"}
```

 Restart your simulation for navigation and try a new navigation order through Rviz.

 Display the **/move_base/global_costmap/costmap** into rviz.

 What happened with the new configuration ? Explain the different robot trajectories ?

### 3.2 Impact of inflate layer parameters on navigation path

Launch the following command:

```
rosrun rqt_reconfigure rqt_reconfigure

```

Play with different values to see the impact on the **/move_base/global_costmap/costmap**


Try to make plan into the map through the make_plan service (move_base package) to see the influence of the parameter variations


E.g of make_plan command
```
rosservice call /move_base/make_plan "start:
  header:
    seq: 0
    stamp:
      secs: 0
      nsecs: 0
    frame_id: 'map'
  pose:
    position:
      x: -4.16
      y: 1.57
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
goal:
  header:
    seq: 0
    stamp:
      secs: 0
      nsecs: 0
    frame_id: 'map'
  pose:
    position:
      x: -4.0
      y: -5.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
tolerance: 0.5"

```


Explain the impact of the parameters. (see [inflate layer](http://wiki.ros.org/costmap_2d/hydro/inflation))


In gazebo, add an obstacle in front of the robot. Try to  send an order of navigation through rviz. What happen ? Why ?


## 4. Obstacle layer


### 4.1 Configuration

1. Copy all configuration files from the param/inflate_layer folder to the param/obstacle_layer folder.
2. Change the launch file as follow to use your new config.
 in the amcl_demo.launch file :

```xml
<include file="$(find turtlebot_gazebo)/launch/navigation/move_base_obstacle_layer.launch.xml">
    <arg name="laser_topic" default="$(arg scan_topic)"/>
  </include>
```


### 4.2 Add an obstacle layer
In the configuration file **costmap_common_params.yaml**, define an inflate layer (follow documentation in costmap_2d) :

```yaml
robot_radius: 0.20
map_type: voxel

static_layer:
  enabled:              true

inflation_layer:
  enabled:              true
  cost_scaling_factor:  5
  inflation_radius:     0.5

#New section
obstacle_layer:
  enabled:              true
  combination_method:   1

  #ObstacleCostmapPlugin
  track_unknown_space:  true    

  #VoxelCostmapPlugin
  origin_z: 0.0         
  z_resolution: 0.2     
  z_voxels: 10  
  unknown_threshold:    15  
  mark_threshold:       0
  publish_voxel_map: false  

  #Sensor management parameter
  max_obstacle_height:  2.5
  obstacle_range: 5.0  
  raytrace_range: 5.0  
  observation_sources: scan  

  #Observation sources
  scan:
    data_type: LaserScan
    topic: /kinect_scan
    marking: true  
    clearing: true  
    min_obstacle_height: 0.25  
    max_obstacle_height: 1.45

```

Find the definition of each parameter ([ros answers](https://answers.ros.org/question/226822/clear-map-around-the-robot/),[obstacle layer](http://wiki.ros.org/costmap_2d/hydro/obstacles)):
 - obstacle_layer:
  - combination_method
  - ObstacleCostmapPlugin
    - track_unknown_space
  - Sensor management parameter
    - observation_sources
    - max_obstacle_height
    - obstacle_range
    - raytrace_range
  - VoxelCostmapPlugin
    - origin_z
    - z_resolution
    - z_voxels
    - unknown_threshold
    - mark_threshold
    - publish_voxel_map

  Note: you can see that the information source come from the Kinect camera and not to the hokuyo. Currently an error style occurs when using hokuyo do the fact that the system says that the collected info is out of range

  - Which kind of information is used as observation source ? Is there any alternative ?

Add the new obstacle layer to the local costmap by modifying the **local_costmap_params.yaml** configuration file:


```yaml
local_costmap:
  global_frame: /odom
  robot_base_frame: /base_footprint

  update_frequency: 5.0
  publish_frequency: 2.0
  static_map: false
  rolling_window: true

  origin_x: 5.0
  origin_y: 5.0
  origin_z: 5.0

  width: 10.0
  height: 10.0
  resolution: 0.05

  transform_tolerance: 0.5
  plugins:
    - {name: obstacle_layer,      type: "costmap_2d::VoxelLayer"}

```

Following the document of the [costmap_2d](http://wiki.ros.org/costmap_2d) explain each parameters.


### 4.3 First test of the obstacle layer

Restart your simulation.

Display the **/move_base/local_costmap/costmap** into rviz.

In gazebo, add an obstacle in front of the robot (visible by the kinect camera).

Try to  send an order of navigation through rviz. What happen ? Why ?


### 4.4 Add a custom inflate layer to the local planner

In order to avoid correctly the obstacle, create a new inflate layer into the **costmap_common_params.yaml**


```yaml
robot_radius: 0.20
map_type: voxel

static_layer:
  enabled:              true

inflation_layer:
  enabled:              true
  cost_scaling_factor:  5
  inflation_radius:     0.5

obstacle_layer:
  enabled:              true
  combination_method:   1

  #ObstacleCostmapPlugin
  track_unknown_space:  true    

  #VoxelCostmapPlugin
  origin_z: 0.0         
  z_resolution: 0.2     
  z_voxels: 10  
  unknown_threshold:    15  
  mark_threshold:       0
  publish_voxel_map: false  

  #Sensor management parameter
  max_obstacle_height:  2.5
  obstacle_range: 5.0  
  raytrace_range: 5.0  
  observation_sources: scan  

  #Observation sources
  scan:
    data_type: LaserScan
    topic: /kinect_scan
    marking: true  
    clearing: true  
    min_obstacle_height: 0.25  
    max_obstacle_height: 1.45

#New section
inflation_local_layer:
  enabled:              true
  cost_scaling_factor:  5  
  inflation_radius:     0.3

```

Update the local planner to take into account this layer, modify the **local_costmap_params.yaml** configuration file:

```yaml
local_costmap:
  global_frame: /odom
  robot_base_frame: /base_footprint

  update_frequency: 5.0
  publish_frequency: 2.0
  static_map: false
  rolling_window: true

  origin_x: 5.0
  origin_y: 5.0
  origin_z: 5.0

  width: 10.0
  height: 10.0
  resolution: 0.05

  transform_tolerance: 0.5
  plugins:
  #CAUTION NEED TO GET OBSTACLE LAYER before inflate layer
    - {name: obstacle_layer,      type: "costmap_2d::VoxelLayer"}
    - {name: inflation_local_layer,     type: "costmap_2d::InflationLayer"}

```

### 4.5 Second test of the obstacle layer

Restart your simulation.

Display the **/move_base/local_costmap/costmap** into rviz.

In gazebo, add an obstacle in front of the robot (visible by the kinect camera).

Try to  send an order of navigation (rviz). What happen ? Why ?


### 4.6 Third test of the obstacle layer

Reduce the size of the local costmap window:


```yaml
local_costmap:
 ...

  origin_x: -2.0
  origin_y: -2.0
  origin_z: -2.0

  width: 4.0
  height: 4.0
  resolution: 0.05

 ...

```

Restart your simulation.

In gazebo, move your robot into a room with 4 doors.


Add an obstacle into rviz blocking the closest door.

Using rviz, ask your robot to navigate to another room avoiding the new obstacle


Does the global planner take into account the obstacle ? Why ?


Ask the robot to navigate far from the blocked door. Then ask it again to reach a goal behind the door blocked by the obstacle.
What happen ? Why ?
How is it possible to fix that ?


### 4.7 Add an obstacle layer to the global costmap

Modify the **global_costmap_params.yaml** configuration file as follow:

```yaml
global_costmap:
   lobal_frame: /map
   robot_base_frame: /base_footprint
   update_frequency: 2.5
   publish_frequency: 2.0
   #static_map: true
   transform_tolerance: 0.5
   plugins:
     - {name: static_layer,            type: "costmap_2d::StaticLayer"}
     - {name: obstacle_layer,      type: "costmap_2d::VoxelLayer"}
     - {name: inflation_layer,         type: "costmap_2d::InflationLayer"}

```


### 4.8 Fourth test of the obstacle layer

Restart your simulation.

In gazebo, move your robot to a room with 4 doors.


Add an obstacle into rviz that block the closest door.

Through rviz ask your robot to navigation to another room avoiding the new obstacle

What happen ? Why ? what is the difference with the new  **/move_base/global_costmap/costmap** into rviz ? Why ?


## 5. Add 3d obstacle detection

### 5.1 Configuration

1. Copy all configuration files from the param/obstacle_layer folder to the param/3d_obstacle_layer folder.
2. Change the launch file as follow to use your new config.
 in the amcl_demo.launch file :

```xml
<include file="$(find turtlebot_gazebo)/launch/navigation/move_base_3d_obstacle_layer.launch.xml">
    <arg name="laser_topic" default="$(arg scan_topic)"/>
  </include>
```

### 5.2 First Test with 3D obstacle
Restart your simulation.

Display the **/move_base/local_costmap/costmap** into rviz.

In gazebo, add a table obstacle in front of the robot.

Ask the robot to navigate behind the table.

What happened ? why ?

Display the PointCloud2 of the **/camera/depth/points** topic into rviz.

What can you conclude ?


### 5.3 Add a source of observation

Go into the **costmap_common_params.yaml** configuration file and add an observation source as follow:

```yaml
robot_radius: 0.20
map_type: voxel

static_layer:
  enabled:              true

#Inflate layer for the global cost map
inflation_layer:
  enabled:              true
  cost_scaling_factor:  5
  inflation_radius:     0.5

obstacle_layer:
  enabled:              true
  combination_method:   1

  #ObstacleCostmapPlugin
  track_unknown_space:  true

  #VoxelCostmapPlugin
  origin_z: 0.0
  z_resolution: 0.2
  z_voxels: 10
  unknown_threshold:    15
  mark_threshold:       0
  publish_voxel_map: false

  #Sensor management parameter
  max_obstacle_height:  1.0 #2.5
  obstacle_range: 5.0
  raytrace_range: 5.0
  observation_sources: scan pcl_scan

  #Observation sources
  scan:
    data_type: LaserScan
    topic: /kinect_scan
    marking: true
    clearing: true
    min_obstacle_height: 0.25
    max_obstacle_height: 1.45

  #New Section
  pcl_scan:
    data_type: PointCloud2
    topic: /camera/depth/points
    marking: true
    clearing: true
    min_obstacle_height: 0.25
    max_obstacle_height: 2.0
    obstacle_range: 2.0
    raytrace_range: 2.5
    inf_is_valid: false

#Inflate layer for the local cost map
inflation_local_layer:
  enabled:              true
  cost_scaling_factor:  5
  inflation_radius:     0.3
```

For a good configuration of the 3D sensor, you must adjust the parameters of the **VoxelCostmapPlugin**. Have a look to the [ROS navigation tuning guide](http://kaiyuzheng.me/documents/navguide.pdf), page 13 to get more detail about Voxel.


### 5.4 Second with 3D obstacle
Restart your simulation.

Display the **/move_base/local_costmap/costmap** into rviz.

In gazebo, add a table obstacle in front of the robot.

Ask the robot to navigate behind the table.

What happened ? why ?
