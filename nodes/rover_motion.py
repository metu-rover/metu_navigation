#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist, Pose, PoseStamped, Point
from leo-rover-locomotion.srv import GetPathFromMap, GetPathFromMapRequest


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
    target = Pose()
    rover = Pose()
    path = Point()
    dist = -1
    u_max = 0.5
    K_p = 0.5

    # initialize node
    rospy.init_node('rover_controller', anonymous=True)

    # publish /cmd_vel to drive rover and
    # initialize its message container
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    msg = Twist()

    # wait until the service point creator to get the next action
    # is ready and get a proxy of it
    rospy.loginfo_once('[rover_controller] is waiting... /get_path_from_map')
    rospy.wait_for_service('/get_path_from_map')
    srv = rospy.ServiceProxy('get_path_from_map', GetPathFromMap)

    # subscribe the topic /pose to learn local position of the rover
    # TODO: rospy.Subscriber('pose', PoseStamped, update_position, rover)

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():

        rospy.loginfo(
            '[rover_controller] is requesting... /get_path_from_map')
        rvr = Point(950, 450, 4)
        trg = Point(170, 500, 4)

        if dist == -1 or dist < 5:
            req = GetPathFromMapRequest(rvr, trg)
            res = srv(req)

        dist = math.sqrt((res.next.x - rover.position.x) **
                         2 + (res.next.y - rover.position.y)**2)

        alpha = math.atan2(res.next.y - rover.position.y,
                           res.next.x - rover.position.x)
        alpha_cur = Quad2Euler(rover.orientation)[2]

        dot_product = (math.cos(alpha) * math.cos(alpha_cur) +
                       math.sin(alpha) * math.sin(alpha_cur))

        msg.linear.x = dot_product * u_max if dot_product > 0 else 0
        msg.angular.z = (alpha_cur - alpha) * K_p

        pub.publish(msg)

        rate.sleep()
