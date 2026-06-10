# Clock Publisher Node

This node publishes the current time as a ROS message of type `rosgraph_msgs/Clock` on the topic `/clock`.
It allows to speed up or slow down the time by a factor `clock_rate` which can be set via a ROS parameter or argument to the launch file.
Make sure to set the parameter `use_sim_time` to `true` in order to use the time published by this node.

Example:
## Terminal 1
```bash
roscore
```

## Terminal 2
```bash
rosparam set use_sim_time true

roslaunch clock_publisher clock.launch clock_rate:=0.1
```

## Terminal 3
Run whatever node you want to use the simulated time. E.g.
```bash
roslaunch crs_launch sim_single_car.launch experiment_name:=pacejka_mt_mpc
```
