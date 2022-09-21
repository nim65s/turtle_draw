# Turtle Draw

Small project to test ROS2 at ANF.

### Turtle Draw

A ROS 2 Node listen for a turtle `pose` topic, and call its `set_pen` service to set the color according to an image:

```
ros2 run turtle_draw turtle_draw
```

### Turtle Sweep

A ROS 2 Node to control a turtle to sweep methodically the whole screen. Usefull to check Turtle Draw.

```
ros2 run turtle_draw turtle_sweep
```

### Draw Launch

A ROS 2 Launcher to start `turtlesim_node`, `turtle_draw`, and `turtle_sweep`

```
ros2 launch turtle_draw draw_launch.py
```


## TODO

We should be able to dynamically change Turtle Draw image by sending it to `/camera/image_raw`
