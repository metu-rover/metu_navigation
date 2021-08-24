#!/usr/bin/env python
import rospy
import actionlib
import numpy as np

from std_msgs.msg import Empty, String
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped, Pose2D
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from nav_msgs.msg import Odometry

import tf2_ros, tf
from tf import transformations as t
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply

ACTIONABLE_DISTANCE = 5
ACTIONABLE_DURATION = rospy.Duration(3.0)
ACTIONABLE_QUANTITY = 20

# STATE
STATE_AUTO = 0
STATE_TELEOP = 1
state = STATE_TELEOP

# COMMAND
COMMAND_START = 2
COMMAND_TELEOP = 3
COMMAND_CANCEL = 4
COMMAND_AUTO = 5

seq = 1

#odom_data
position = Pose2D()
epsilon = Pose2D()

def move_base_incr_seq(msg):
    global seq
    rospy.loginfo('goal detected')
    seq += 1

def move_base_send_goal(waypoint_name):
    global seq
    goal = MoveBaseGoal()
    
    waypoint = waypoints[waypoint_name]
    x, y, theta = waypoint['x'], waypoint['y'], waypoint['theta']
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
    
    rospy.loginfo("next waypoint is %s at x:%2.1f y:%2.1f theta:%2.1f" % (waypoint_name, x, y, theta))


def move_base_cancel():
    rospy.loginfo("rover stopped")
    client.cancel_goal()


def callback_control(msg, args):
    command = args
    global state

    if command == COMMAND_TELEOP:
        state = STATE_TELEOP
        rospy.loginfo("autonomous navigation paused")
    elif command == COMMAND_AUTO:
        state = STATE_AUTO
        rospy.loginfo("autonomous navigation resumed")
    elif command == COMMAND_CANCEL:
        state = STATE_TELEOP
        move_base_cancel()
    elif command == COMMAND_START:
        move_base_send_goal(curr_goal)


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

def list_from_pose2d(pose2d):
    return list(map(lambda attr: pose2d.__getattribute__(attr), ('x','y', 'theta')))


def update_state_by_ar_marker(ar_marker_detection):
    global state, curr_goal, next_goal, position, epsilon
    now = rospy.Time.now()
    for marker_number, detections in ar_marker_detection.items():
    
    
        while len(detections) != 0 and now - detections[0][1] > ACTIONABLE_DURATION:
            detections.pop(0)
        if len(detections) > ACTIONABLE_QUANTITY:
            print(marker_number, detections[0][0])
            marker = Pose2D(*np.mean([p for p,_ in detections], axis=0))
        #pub.publish(marker)
            rospy.loginfo('Pose 2D x:{} y:{} theta:{}'.format(*list_from_pose2d(marker)))
            rospy.loginfo('{}'.format(*map(lambda detection:detection[0], detections)))
            
            epsilon.x = marker.x - position.x
            epsilon.y = marker.y - position.y
            epsilon.theta = marker.theta - position.theta


def update_odom_data(msg, pub): #: PoseWithCovarianceStamped
    global position, epsilon
    position.x = epsilon.x + msg.pose.pose.position.x
    position.y = epsilon.y + msg.pose.pose.position.y
    position.theta = epsilon.theta + euler_from_quaternion(list_from_rotation(msg.pose.pose.orientation))[2]
    pub.publish(position)


def norm(p1, p2=None):
    try:
        if p2:
            p2 = np.zeros(0)
    except ValueError:
        pass
    return np.sum(np.sqrt(np.square(p1 - p2)))


def callback_marker_detected(msg):
    global positon
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

                rospy.loginfo_throttle(5, "{} detected x:{:.2f} y:{:.2f} z:{:.2f} away from the rover"
                        .format(marker_number, *translation))
                
                rospy.loginfo_throttle(5, "relative orientation roll: {:.2f} pitch: {:.2f} yaw: {:.2f}"
                        .format(*euler_from_quaternion(quaternion)))
                
                if np.sum(np.sqrt(np.square(translation))) < ACTIONABLE_DISTANCE:

                    # getting pre-defined positions for ar_marker_#
                    ar_tag_absolute = ar_marker_positions[marker_number]
                    ar_tag_translation = np.array(map(lambda key: float(ar_tag_absolute[key]), ('x','y','z')))

                    yaw = (ar_tag_absolute['yaw'] - euler_from_quaternion(quaternion)[2]) / 3.14 * 180
            
                    e_translation = np.multiply(np.take(translation, [1, 0, 2]), [-1, +1, +1])
                    w_translation = np.multiply(np.take(translation, [1, 0, 2]), [+1, -1, +1])
                    n_translation = np.multiply(np.take(translation, [0, 1, 2]), [+1, +1, +1]) 
                    s_translation = np.multiply(np.take(translation, [0, 1, 2]), [-1, -1, +1])

                    p1 = ar_tag_translation - np.array(list_from_pose2d(position))
                    
                    translations = (e_translation, w_translation, n_translation, s_translation)
                    directions   = ('east', 'west', 'north', 'south')
                    
                    idx = np.argmin(map(lambda __translation: norm(p1, __translation), translations))
                    translation = translations[idx]
                    direction = directions[idx]
                    
                    rospy.loginfo_throttle(5, "the rover's position has updated by x:{:.2f} y:{:.2f} z:{:.2f} on side"
                            .format(*(ar_tag_translation - translation)) + direction)

                    
                    rospy.loginfo("south x:{:.2f} y:{:.2f} z:{:.2f}"
                            .format(*(ar_tag_translation - s_translation)))
                    rospy.loginfo("north x:{:.2f} y:{:.2f} z:{:.2f}"
                            .format(*(ar_tag_translation - n_translation)))
                    rospy.loginfo("east  x:{:.2f} y:{:.2f} z:{:.2f}"
                            .format(*(ar_tag_translation - e_translation)))
                    rospy.loginfo("west  x:{:.2f} y:{:.2f} z:{:.2f}"
                            .format(*(ar_tag_translation - w_translation)))

                    rospy.loginfo('direction {}'.format(direction))


                    ar_marker_detections[marker_number].append((ar_tag_translation - translation, rospy.Time.now()))

                    if len(ar_marker_detections[marker_number]) > ACTIONABLE_QUANTITY:
                        update_state_by_ar_marker(ar_marker_detections)

                else:
                    far_marker_count += 1

        except (tf2_ros.ExtrapolationException, tf2_ros.LookupException, tf2_ros.ConnectivityException):
            pass
    
 

def get_param(key):
    while True:
        try:
            return rospy.get_param(key)
        except KeyError:
            rospy.logerr('param {} not found'.format(key))
            rospy.sleep(2)

#odom_pub = None

def odom_publisher_callback(event=None):
    global position, odom_pub
    msg = Odometry()
    msg.pose.pose.position.x = position.x
    msg.pose.pose.position.y = position.y
    x,y,z,w = quaternion_from_euler(0, 0, position.theta)
    msg.pose.pose.orientation.x = x
    msg.pose.pose.orientation.y = y
    msg.pose.pose.orientation.z = z
    msg.pose.pose.orientation.w = w
    odom_pub.publish(msg) 
    





if __name__ == '__main__':
    rospy.init_node('rover_autonomous_node', anonymous=True)
    
    waypoints = get_param('waypoints')

    ar_marker_waypoints = get_param('actions/ar_markers')
    ar_marker_positions = get_param('tf_static')
    next_points = get_param('remote_path')

    ar_marker_detections = {ar_marker: [] for ar_marker in ar_marker_positions.keys()}
        
    curr_goal = 'start'
    next_goal = next_points[curr_goal]
    
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    pub = rospy.Publisher('base_link_position', Pose2D, queue_size=10)
    rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, update_odom_data, queue_size=10, callback_args=pub)
    rospy.wait_for_message('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped)
    rospy.loginfo('successfuly subscribed /robot_pose_ekf/odom_combined')

    
    rospy.Subscriber('/ar_pose_marker', AlvarMarkers, callback_marker_detected)
    
    
    rospy.loginfo(rospy.get_name() + " successfully started")
    
    rospy.Subscriber('automate', Empty, callback_control, callback_args=COMMAND_AUTO)
    rospy.Subscriber('start',  Empty, callback_control, callback_args=COMMAND_START)
    rospy.Subscriber('teleop', Empty, callback_control, callback_args=COMMAND_TELEOP)
    rospy.Subscriber('cancel', Empty, callback_control, callback_args=COMMAND_CANCEL)
    
    rospy.Subscriber('next', Empty, callback_step, callback_args=+1)
    rospy.Subscriber('prev', Empty, callback_step, callback_args=-1)
    rospy.Subscriber('goto', String, callback_set)
    
    rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, move_base_incr_seq)

    odom_pub = rospy.Publisher('marker_odom', Odometry, queue_size=10)

    rospy.Timer(rospy.Duration(1.0/30), odom_publisher_callback)

    while not rospy.is_shutdown():
        if state != STATE_AUTO:
            rospy.wait_for_message('automate', Empty)
        
        curr_goal = next_goal
        next_goal = next_points[curr_goal]
        move_base_send_goal(curr_goal)
        client.wait_for_result()

