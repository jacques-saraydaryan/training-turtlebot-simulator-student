# Overview of different folder
- /worlds : set of worlds for simulation into gazebo
- /maps: set of maps corresponding to the different simulated worlds. Allow to do navigation
- /params: set of param files especially for navigation purpose. all different configuration for the navigation are set here
- /behavior_tree: set of Behavior tree  description used for robot navigation strategy

# Environment (Docker container of the course)
- Set up the environment by following the Docker guide **[SETUP_TP.md](https://gitlab.com/cpelyon/rob_ressources/ros2_cuda_gazebo.docker/-/blob/main/SETUP_TP.md)** (steps 0 to 4): Docker image, workspace, submission repository, container launch.
  - Do **not** `git clone` this repository: add it to your submission repository with `git subtree add` (guide, step 2, `tp_nav`).
- Everything is already installed in the container (Nav2, Turtlebot simulation, cyclonedds, image pipeline) and the ROS environment is already set (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, ROS limited to your machine, `TURTLEBOT3_MODEL=waffle`): no `apt install` or `export` needed.
- Launch the container from the root of your workspace, and run it again to open other terminals in the same container:
```
    ./run_tp.sh
```
- In the container, compile your workspace and set the path of this training repository:
```
    cd /root/student_ws
    colcon build --symlink-install
    source install/setup.bash
    export TRAINING_NAV=/root/student_ws/src/training-turtlebot-simulator-student
```

# Start Simlation for mapping

```
    ros2 launch nav2_bringup tb4_simulation_launch.py headless:=False slam:=True  world:="$TRAINING_NAV/training-nav2-overview/worlds/baseline.world" x_pose:=0 y_pose:=0 z_pose:=0
```

```
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

- Save your map in your workspace (anything outside `/root/student_ws` is lost when the container stops):
```
    mkdir -p $TRAINING_NAV/training-nav2-overview/maps
    ros2 run nav2_map_server map_saver_cli -f $TRAINING_NAV/training-nav2-overview/maps/<your map>
```

# Start Simlation for navigation

- Launch Gazebo, Localization, navigation and tools.


```
    ros2 launch nav2_bringup tb4_simulation_launch.py headless:=False params_file:="$TRAINING_NAV/training-nav2-overview/params/nav2_params_empty.yaml" map:="$TRAINING_NAV/training-nav2-overview/maps/<your map>.yaml" world:="$TRAINING_NAV/training-nav2-overview/worlds/baseline.world" x_pose:=0 y_pose:=0 z_pose:=0
```

- On rviz set "2d pose estimate" 

- If you need RGBD camera launch the following command:

```
    ros2 run depth_image_proc point_cloud_xyz_node --ros-args -r image_rect:=/rgbd_camera/depth_image -r camera_info:=/rgbd_camera/camera_info -r points:=/camera/depth/points
```

