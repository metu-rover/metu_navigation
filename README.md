# leo-rover-localization

to run bot robot_localization package

```shell
roslaunch leo_rover_localization robot_localization.launch
```

to run rover_controller_node.py and path_planner_service.py

```shell
roslaunch leo_rover_localization robot_controller.launch
```

to allow rover_controller_node.py publishing to cmd_vel
```shell
rosservice call /leo_locomotion/enable_motors "enable: True"
```

to update the rover with a new destination
```shell
rosservice call /leo_locomotion/set_destination "destination:
  x: 0.0
  y: 0.0
  theta: 0.0" 
```
and make sure that you get the `resopnd: true`
