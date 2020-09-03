# metu-rover
MetuMech - Rover Team 4 ERC

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

to set waypoint
```shell
rosservice call /set_waypoints "enable: True"
```