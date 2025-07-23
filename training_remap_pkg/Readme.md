# Test Node behavior

- Start Node in debug:

```
ros2 run training_remap_pkg remapper --ros-args --log-level debug
```

- Publish a node on the default incoming topic

```
ros2 topic pub /cmd_vel_fake geometry_msgs/msg/Twist "{'linear': {'x':0.0,'y':0.0,'z':0.0},'angular':{'x':0.0,'y':0.0,'z':0.0}}" -r 10
```

- Start the node with redefined topics

```
ros2 run training_remap_pkg remapper --ros-args -p input_topic:=/cmd_vel_fake2 -p output_topic:=/cmd_vel_smoothed2
```
