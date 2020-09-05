#!/usr/bin/env python
import math
import rospy
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose2D
from nav_msgs.msg import Odometry


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


def callback_localization(msg, odm):
    odm.pose.pose.position.x = msg.pose.pose.position.x
    odm.pose.pose.position.y = msg.pose.pose.position.y
    odm.pose.pose.position.z = msg.pose.pose.position.z
    odm.pose.pose.orientation.x = msg.pose.pose.orientation.x
    odm.pose.pose.orientation.y = msg.pose.pose.orientation.y
    odm.pose.pose.orientation.z = msg.pose.pose.orientation.z
    odm.pose.pose.orientation.w = msg.pose.pose.orientation.w


def callback_artag_marker(msg, args):
    odm, cum = args

    cum.x = msg.transform.translation.x - odm.pose.pose.position.x
    cum.y = msg.transform.translation.y - odm.pose.pose.position.y
    cum.theta = Quad2Euler(msg.transform.rotation)[2] - \
        Quad2Euler(odm.pose.pose.orientation)[2]


if __name__ == '__main__':
    rospy.init_node('rover_localization', anonymous=True)

    total = Pose2D()
    odm = Odometry()
    msg = Pose2D()

    # rufat's node
    rospy.Subscriber('odometry/filtered', Odometry, callback_localization, odm)

    # Arda's node
    rospy.Subscriber('transform', TransformStamped,
                     callback_artag_marker, (odm, total))


    pub = rospy.Publisher('gridpose', Pose2D, queue_size=10)

    rospy.wait_for_message('odometry/filtered', Odometry, timeout=60)

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        msg.x = total.x + odm.pose.pose.position.x
        msg.y = total.y + odm.pose.pose.position.y
        msg.theta = total.theta + Quad2Euler(odm.pose.pose.orientation)[2]
        pub.publish(msg)
        
        rate.sleep()
