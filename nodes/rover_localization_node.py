#!/usr/bin/env python
import math
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose2D, Twist, TwistStamped
from leo_rover_localization.srv import SetReferancePose, SetDestinationResponse
from nav_msgs.msg import Odometry

epsilon = 0.05


def Quad2Euler(q):
    """
    This function transforms from quernion to euler

    #Quaternion_to_Euler_Angles_Conversion
    For details, see https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

    Parameters
    ----------
    q : Pose().orientation
        This is an orientation vector in queternion form

    Returns
        The euler form of the parameter q
    -------
    int
        Description of return value
    """
    # roll (x-axis rotation)
    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2 * (q.w * q.y - q.z * q.x)
    if (abs(sinp) >= 1):
        # use 90 degrees if out of range
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    # Transform the q of the form PoseStamped().pose.orientation to angle
    # by applying atan2(y,x) => float angle from math library.
    return roll, pitch, yaw


def callback_localization(msg, args):
    rover, mover = args
    rover.x = msg.pose.pose.position.x - mover.x
    rover.y = msg.pose.pose.position.y - mover.y
    rover.theta = Quad2Euler(msg.pose.pose.orientation)[2] - mover.theta


def callback_locomotion(msg, vel):
    vel.linear.x = msg.twist.linear.x
    vel.angular.z = msg.twist.angular.z


def callback_artag_marker(msg, args):
    odm, cum, vel = args

    if abs(vel.linear.x) < epsilon and abs(vel.angular.z) < 2 * epsilon:
        cum.x = msg.transform.translation.x - odm.x
        cum.y = msg.transform.translation.y - odm.y
        # rufat - odometry
        # rufat_0
        # arda - transform ar_tag
        # total


def handle_taring_the_balance(msg):
    callback_artag_marker()


if __name__ == '__main__':
    rospy.init_node('rover_localization', anonymous=True)

    msg = Pose2D()  # msg
    map_frame_pose = Pose2D()  # rover
    total = Pose2D()  # total
    reference = Pose2D()  # balance
    vel = Twist()

    # rufat's node
    rospy.Subscriber('odometry/filtered', Odometry,
                     callback_localization, (map_frame_pose, reference))

    # Arda's node
    rospy.Subscriber('/base_link_transform', TransformStamped,
                     callback_artag_marker, (map_frame_pose, total, vel))
    rospy.Subscriber('/wheel_odom', TwistStamped, callback_locomotion, vel)
    rospy.Service('taring_the_balance', SetReferancePose,
                  handle_taring_the_balance)

    pub = rospy.Publisher('ground_truth_to_pose2D', Pose2D, queue_size=10)

    rospy.wait_for_message(
        '/leo_localization/odometry/filtered', Odometry, timeout=10)
    rospy.loginfo_once('[rover_localization] connected odometry/filtered')
    reference.x = map_frame_pose.x
    reference.y = map_frame_pose.y
    reference.theta = map_frame_pose.theta

    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        # init + A_0 + (R_cur - R_0)
        msg.x = total.x + map_frame_pose.x
        msg.y = total.y + map_frame_pose.y
        msg.theta = map_frame_pose.theta

        pub.publish(msg)

        rate.sleep()
