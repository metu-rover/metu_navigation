#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Pose, Twist


def Quad2Euler(q):
    """
    This function transforms from quernion to euler

    For details, see https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Quaternion_to_Euler_Angles_Conversion

    Parameters
    ----------
    q : PoseStamped().pose.orientation
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
    return (roll, pitch, yaw)


def callback_function (msg):
    rospy.loginfo('x:%2.1f y:%2.1f' % (msg.position.x, msg.position.y))


if __name__ == '__main__':

    rospy.init_node('subscriber', anonymous=True)

    rospy.Subscriber('/turtle1/pose', Pose, callback_function)

    rospy.spin()
