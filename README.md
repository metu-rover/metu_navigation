# metu-rover

to run service_node.py

```shell
roslaunch leo_rover_localization service.launch
```

to run robot_localization package

```shell
roslaunch leo_rover_localization robot_localization.launch
```

to update a new waypoint to rover
```shell
rosservice call /set_waypoint "target:
  x: 10.0
  y: 0.0
  z: 0.0" 
```
and make sure that you get the `resopnd: True`
