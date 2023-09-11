# Overview of different folder
- /worlds : set of worlds for simulation into gazebo
- /maps: set of maps corresponding to the different simulated worlds. Allow to do navigation
- /params: set of param files especially for navigation purpose. all different configuration for the navigation are set here

# Start Simlation for mapping

```
ros2 launch gazebo_sim_nav tb3_simulation_without_nav_launch.py headless:=False
ros2 launch slam_toolbox online_async_launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```


# Start Simlation for navigation

- Launch Gazebo, Localization, navigation and tools.

```
cd src/training-turtlebot-simulator-student/simulation/gazebo/gazebo_sim_nav/params/
ros2 launch gazebo_sim_nav tb3_simulation_launch.py headless:=False params_file:="nav2_params_empty.yaml" map:="<your absolute map path>/<your map>.yaml"
```
