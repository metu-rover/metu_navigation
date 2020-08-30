#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist, Pose, PoseStamped, Point
from ros_service.srv import ServiceExample


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
    return (roll, pitch, yaw)


def update_position(msg, rover):
    rover.position.x = msg.position.x
    rover.position.y = msg.position.y
    rover.position.z = msg.position.z
    rover.orientation.x = msg.orientation.x
    rover.orientation.y = msg.orientation.y
    rover.orientation.z = msg.orientation.z
    rover.orientation.w = msg.orientation.w


if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True)

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    me = ''
    next_point = Point()
    rover = Pose()

    # BUG: handler is not working
    service = rospy.Service('point_creator', ServiceExample, handler=me)

    # TODO: find the topic of the agent
    rospy.Subscriber('pose', PoseStamped, update_position, rover)
    u_max = 0.5
    K_p = 0.5

    msg = Twist()

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        alpha = math.atan2(next_point.y - rover.position.y,
                           next_point.x - rover.position.x)
        alpha_cur = Quad2Euler(rover.orientation)[2]

        dot_product = (math.cos(alpha) * math.cos(alpha_cur) +
                       math.sin(alpha) * math.sin(alpha_cur))
                       
        msg.linear.x = dot_product * u_max if dot_product > 0 else 0
        msg.angular.z = (alpha_cur - alpha) * K_p

        pub.publish(msg)

        rate.sleep()
