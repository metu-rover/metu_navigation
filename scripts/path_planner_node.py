#!/usr/bin/env python
import rospy
import actionlib
import numpy as np

from std_msgs.msg import Empty, String
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped, Pose2D
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers

import tf2_ros, tf
from tf import transformations as t
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply

ACTIONABLE_DISTANCE = 5
ACTIONABLE_DURATION = rospy.Duration(3.0)
ACTIONABLE_QUANTITY = 7

# STATE
AUTOMATED = 0
MANUAL = 1

# COMMAND
PAUSE = 2
RESUME = 3
STOP = 4

state = MANUAL

client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
seq = 1

def move_base_send_goal(waypoint_name):
    global seq
    waypoint = waypoints[waypoint_name]
    x, y, theta = waypoint['x'], waypoint['y'], waypoint['theta']
    rospy.loginfo("next waypoint is %s at x:%2.1f y:%2.1f theta:%2.1f" % (waypoint_name, x, y, theta))
    goal = MoveBaseGoal()
    quaternion = quaternion_from_euler(0, 0, theta)
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.header.seq = seq
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.x = quaternion[0]
    goal.target_pose.pose.orientation.y = quaternion[1]
    goal.target_pose.pose.orientation.z = quaternion[2]
    goal.target_pose.pose.orientation.w = quaternion[3]
    client.send_goal(goal=goal)
    seq += 1


def move_base_cancel():
    rospy.loginfo("rover stopped")
    client.cancel_goal()


def callback_control(msg, args):
    command = args
    global state

    state = AUTOMATED if command == RESUME else MANUAL

    if command == PAUSE:
        rospy.loginfo("autonomous navigation paused")
    elif command == RESUME:
        move_base_send_goal(curr_goal)
        rospy.loginfo("autonomous navigation resumed")
    elif command == STOP:
        move_base_cancel()


def callback_step(msg, args):
    step = args
    global state, curr_goal, next_goal
    while 0 < step:
        curr_goal = next_goal
        next_goal = next_points[curr_goal]
        step -= 1

    while step < 0:
        next_goal = curr_goal
        curr_goal = list(next_points.values()).index(next_goal)
        curr_goal = list(next_points.keys())[curr_goal]
        step += 1

    move_base_send_goal(curr_goal)


def callback_set(msg):
    global curr_goal, next_goal
    if msg.data not in waypoints.keys():
        rospy.logerr("waypoint %s is not in the waypoint list" % (msg.data,))
    else:
        curr_goal = msg.data
        next_goal = next_points[msg.data]
        move_base_send_goal(msg.data)


def list_from_translation(translation):
    return list(map(lambda attr: translation.__getattribute__(attr), ('x','y','z')))

            
def list_from_rotation(rotation):
    return list(map(lambda attr: rotation.__getattribute__(attr), ('x','y','z','w')))


def update_state_by_ar_marker(ar_marker_detection):
    global state, curr_goal, next_goal
    now = rospy.Time.now()
    for marker_number, detections in ar_marker_detection.items():
        while len(detections) != 0 and now - detections[0][1] > ACTIONABLE_DURATION:
            detections.pop(0)
        if len(detections) > ACTIONABLE_QUANTITY:
            pub.publish(Pose2D(*np.mean([p for p,_ in detections], axis=0)))
            rospy.logdebug('Pose 2D x:{} y:{} theta:{}'.format(*np.mean([p for p,_ in detections], axis=0)))


            if marker_number in ar_marker_waypoints.keys() and curr_goal in ar_marker_waypoints[marker_number]:
                curr_goal = next_goal
                next_goal = next_points[curr_goal]
                print(marker_number, ar_marker_waypoints[marker_number])

                if state == AUTOMATED:
                    move_base_send_goal(curr_goal)


def callback_marker_detected(msg):
    (total_base_x, total_base_y, total_base_t, far_marker_count) = (0, 0, 0, 0)
    detected_marker_numbers = ['ar_marker_' + str(marker.id) for marker in msg.markers if 1 <= int(marker.id) <= 15]
    
    for marker_number in detected_marker_numbers:
        try:
            # if tf from map to ar_marker_# does exist 
            if tf_buffer.can_transform('zed2_tilt_head_link', marker_number, rospy.Time()):
                # then lookup the transform
                ar_tag_relative = tf_buffer.lookup_transform('base_link', marker_number, 
                                                             rospy.Time(), rospy.Duration(3.0))
                # finding transform from zed2_tilt_head_link to ar_marker_#
                translation = list_from_translation(ar_tag_relative.transform.translation)
                quaternion  = list_from_rotation(ar_tag_relative.transform.rotation)

                rospy.loginfo_throttle(5, "{} detected relatively at x:{} y:{} z:{}".format(marker_number, *translation))
                
                if np.sum(np.sqrt(np.square(translation))) < ACTIONABLE_DISTANCE:
                    # getting pre-defined positions for ar_marker_#
                    if marker_number == 'ar_marker_5':
                        print(np.sum(np.sqrt(np.square(translation))))
                    ar_tag_absolute = ar_marker_positions[marker_number]
                    ar_tag_translation = np.array(map(lambda key: float(ar_tag_absolute[key]), ('x','y','z')))

                    total_base_x += ar_tag_translation[0] - translation[0]
                    total_base_y += ar_tag_translation[1] - translation[1]
                    total_base_t += -euler_from_quaternion(quaternion)[2]
                    
                    rospy.loginfo_throttle(5, "updated position to x:{} y:{} z:{}".format(*(ar_tag_translation - translation)))

                    ar_marker_detections[marker_number].append((ar_tag_translation - translation, rospy.Time.now()))

                    if len(ar_marker_detections[marker_number]) > ACTIONABLE_QUANTITY:
                        update_state_by_ar_marker(ar_marker_detections)

                else:
                    far_marker_count += 1

        except (tf2_ros.ExtrapolationException, tf2_ros.LookupException, tf2_ros.ConnectivityException):
            pass

    if len(detected_marker_numbers) != far_marker_count and 1 <= len(detected_marker_numbers):
        total_base_x /= (len(detected_marker_numbers) - far_marker_count)
        total_base_y /= (len(detected_marker_numbers) - far_marker_count)
        total_base_t /= (len(detected_marker_numbers) - far_marker_count)
    

def get_param(key):
    rate = rospy.Rate(0.5)
    while True:
        try:
            return rospy.get_param(key)
        except KeyError:
            rospy.logerr('param {} not found'.format(key))
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('rover_autonomous_node', anonymous=True)
    
    waypoints = get_param('waypoints')

    ar_marker_waypoints = get_param('actions/ar_markers')
    ar_marker_positions = get_param('tf_static')
    next_points = get_param('remote_path')

    curr_goal = next_points['start']
    next_goal = next_points[curr_goal]
    ar_marker_detections = {ar_marker: [] for ar_marker in ar_marker_positions.keys()}
    
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    #rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, update_odom_data, queue_size=10)
    #rospy.wait_for_message('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped)
    rospy.loginfo('successfuly subscribed /robot_pose_ekf/odom_combined')
    rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback_marker_detected)
    pub = rospy.Publisher('base_link_position', Pose2D, queue_size=10)
    
    rospy.wait_for_message('start', Empty)
    rospy.loginfo("autonomous navigation started")
    
    rospy.Subscriber('next', Empty, callback_step, callback_args=+1)
    rospy.Subscriber('prev', Empty, callback_step, callback_args=-1)
    rospy.Subscriber('resume', Empty, callback_control, callback_args=RESUME)
    rospy.Subscriber('pause', Empty, callback_control, callback_args=PAUSE)
    rospy.Subscriber('stop', Empty, callback_control, callback_args=STOP)
    rospy.Subscriber('waypoint', String, callback_set)
    
    move_base_send_goal(curr_goal)

    rospy.spin()

    rate = rospy.Rate(0.2)
    pub_photo = rospy.Publisher('/save', Empty, queue_size=10)

    while not rospy.is_shutdown():
        if STATE != MANUEL:
            pub_photo.publish(Empty())
        rate.sleep()
        
