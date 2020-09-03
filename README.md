# metu-rover

to run rover_controller_node.py and path_planner_service.py

```shell
roslaunch leo_rover_localization robot_controller.launch
```

to run robot_localization package

```shell
roslaunch leo_rover_localization robot_localization.launch
```

to allow rover_controller_node.py publishing to cmd_vel
```shell
rosservice call /enable_motors "enable: True"
```

to update a new waypoint to rover
```shell
rosservice call /set_waypoint "target:
  x: 10.0
  y: 0.0
  z: 0.0" 
```
and make sure that you get the `resopnd: True`