# Work in progress

- command to lauch to get world in new gazebo
```
ros2 launch nav2_bringup tb4_simulation_launch.py headless:=False params_file:="/home/tp/ros_ws/src/training-turtlebot-simulator-student/simulation/gazebo/gazebo_sim_nav/params/nav2_baseline_params.yaml" map:="/home/tp/ros_ws/src/training-turtlebot-simulator-student/simulation/gazebo/gazebo_sim_nav/maps/js_custom_map_local2.yaml" world:="/home/tp/ros_ws/src/training-turtlebot-simulator-student/simulation/gazebo/gazebo_sim_nav/worlds/baseline.world" 
```

- Other more complete command
    ```
        ros2 launch nav2_bringup tb4_simulation_launch.py headless:=False params_file:="/home/tp/ros_ws/src/training-turtlebot-simulator-student/simulation/gazebo/gazebo_sim_nav/params/nav2_baseline_params.yaml" map:="/home/tp/ros_ws/src/training-turtlebot-simulator-student/simulation/gazebo/gazebo_sim_nav/maps/js_custom_map_local2.yaml" world:="/home/tp/ros_ws/src/training-turtlebot-simulator-student/simulation/gazebo/gazebo_sim_nav/worlds/baseline.world" x_pose:=0 y_pose:=0 z_pose:=0
    ```
- Convert Image depht to PCL
    ```
        ros2 run depth_image_proc point_cloud_xyz_node --ros-args -r image_rect:=/rgbd_camera/depth_image -r camera_info:=/rgbd_camera/camera_info -r points:=/camera/depth/points
    ```


# Overview of different folder
- /worlds : set of worlds for simulation into gazebo
- /maps: set of maps corresponding to the different simulated worlds. Allow to do navigation
- /params: set of param files especially for navigation purpose. all different configuration for the navigation are set here

# Prerequistes
- cyclonedds
```
   sudo apt install ros-jazzy-rmw-cyclonedds-cpp
```
- Nav2
```
    sudo apt install ros-jazzy-navigation2
    sudo apt install ros-jazzy-nav2-bringup
```
- Turtlebot simulation
```
    sudo apt install ros-jazzy-turtlebot3-gazebo
```

- Tools for PCL
```
    sudo apt install ros-jazzy-image-pipeline
```

# Start configuration
- Configure your ROS to communicate only on localhost
```
export ROS_LOCALHOST_ONLY=1
```  
- Some issues as been identified with default dss use cyclonedds instead
```
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```
- set default end directory 

```
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/jazzy/share/turtlebot3_gazebo/models
```
- Tips : Save all these commands into your `~/.bashrc` file

# Start Simlation for mapping

```
ros2 launch gazebo_sim_nav tb3_simulation_without_nav_launch.py headless:=False
```

```
ros2 launch slam_toolbox online_async_launch.py
```

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

# Start Simlation for navigation

- Launch Gazebo, Localization, navigation and tools.

```
cd src/training-turtlebot-simulator-student/simulation/gazebo/gazebo_sim_nav/params/
ros2 launch gazebo_sim_nav tb3_simulation_launch.py headless:=False params_file:="nav2_params_empty.yaml" map:="<your absolute map path>/<your map>.yaml"
```

- On rviz set "2d pose estimate" 