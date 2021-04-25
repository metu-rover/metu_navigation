# Metu-navigation

### GAZEBO-SIM
```shell
roslaunch metu_gazebo metu_maze.launch
```

### MAPPING
```shell
roslaunch metu_navigation rtabmap.launch
```

### NAVIGATION-STACK
```shell
roslaunch metu_navigation move_base.launch
```

### RVIZ-SIM
```shell
roslaunch metu_gazebo rviz.launch
```

METU ROVER OPERATING COMMANDS

To start rtabmap:
```shell
roslaunch rtabmap_ros rtabmap.launch rtabmap_args:= rgb_topic:=/zed2/left/image_rect_color depth_topic:=/zed2/depth/depth_registered camera_info_topic:=/zed2/depth/camera_info odom_topic:=/controllers/diff_drive/odom visual_odometry:=false frame_id:=base_link odom_frame_id:=odom approx_sync:=false rgbd_sync:=true
```
In rviz:
```shell
press the add button and choose robot model
```
To start move_base:
```shell
roslaunch metu_navigation move_base.launch
```
If keyboard plugin is not installed:
```shell
sudo apt-get install ros-melodic-teleop-twist-keyboard
```
To move rover with keyboard:
```shell
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
To check the process from rqt:
```shell
rosrun rqt_tf_tree rqt_tf_tree
```
