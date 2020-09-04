# metu-rover

to run bot robot_localization package, controller_node.py, and path_planner_service

```shell
roslaunch leo_rover_localization robot_localization.launch
```

- to run rover_controller_node.py and path_planner_service.py

  ```shell
  roslaunch leo_rover_localization robot_controller.launch
  ```

to allow rover_controller_node.py publishing to cmd_vel
```shell
rosservice call /enable_motors "enable: True"
```

to update the rover with a new destination
```shell
rosservice call /set_destination "target:
  x: 10.0
  y: 0.0
  z: 0.0" 
```
and make sure that you get the `resopnd: True`
