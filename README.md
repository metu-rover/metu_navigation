# leo-rover-localization

## Launching

Running rover_localization_node.py and ekf_localization_node
```shell
roslaunch leo_rover_localization robot_localization.launch
```

Running rover_locomotion_node.py, rover_controller_node.py, and path_planner_service.py
```shell
roslaunch leo_rover_localization robot_locomotion.launch
```

## Package Interface via ros messages

Enabling autonomous driving, i.e. allow rover_locomotion_node.py publishing to `/cmd_vel`
```shell
rostopic pub -1 /leo_locomotion/rover_listener std_msgs/String "data :'enable motors'"
```

Disabling autonomous driving, i.e. block rover_locomotion_node.py publishing to `/cmd_vel`
```shell
rostopic pub -1 /leo_locomotion/rover_listener std_msgs/String "data :'disable motors'"
```

Setting the desination a run-time defined value from console
```shell
rostopic pub -1 /leo_locomotion/rover_listener std_msgs/String "data :'set_destination <float> <float>'"
```

Setting the desination to a launch-time defined value from `params/waypoints.yaml`
```shell
rostopic pub -1 /leo_locomotion/rover_listener std_msgs/String "data :'set_waypoint <two_characters>'"
```

Setting the position to a launch-time defined value from `params/waypoints.yaml`
```shell
rostopic pub -1 /leo_locomotion/rover_listener std_msgs/String "data :'set_pose <two_characters>'"
```

## Package Interface via ros services

Enabling autonomous driving, i.e. allow rover_locomotion_node.py publishing to `/cmd_vel`
```shell
rosservice call /leo_locomotion/enable_motors "enable: true"
```

Disabling autonomous driving, i.e. block rover_locomotion_node.py publishing to `/cmd_vel`
```shell
rosservice call /leo_locomotion/enable_motors "enable: false"
```

Setting the desination a run-time defined value from console
```shell
rosservice call /leo_locomotion/set_destination "destination:
  x: <float>
  y: <float>
  theta: <float> 
```
and make sure that you get the `response: true`

Setting the position a run-time defined value from console
```shell
rosservice call /leo_localization/taring_the_balance "reference:
  x: <float>
  y: <float>
  theta: <float> 
```
and make sure that you get the `response: true`
