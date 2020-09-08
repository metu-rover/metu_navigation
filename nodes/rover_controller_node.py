#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist, Pose, PoseStamped, Pose2D
from leo_rover_localization.srv import GetPathFromMap, GetPathFromMapRequest
from leo_rover_localization.srv import SetMotorEnable
from leo_rover_localization.srv import SwitchWaypoint, SwitchWaypointRequest
from leo_rover_localization.srv import SetDestination

epsilon = 0.25


def update_position(msg, rover):
    rover.x = msg.x
    rover.y = msg.y
    rover.theta = msg.theta


def handle_set_destination(msg):
    global rover, distance
    req4GetPath = GetPathFromMapRequest(rover, msg.destination)
    res4GetPath = srv4GetPath(req4GetPath)

    if res4GetPath.is_path_updated:
        global target
        target.x = msg.destination.x
        target.y = msg.destination.y
        target.theta = msg.destination.theta
        distance = 0

    return res4GetPath.is_path_updated


def handle_enable_motors(msg):
    global is_enable
    global pub
    is_enable = msg.enable

    if is_enable:
        return "motors are enabled, Don't panic!"
    else:
        pub.publish(Twist())
        return "motors are disabled."


if __name__ == '__main__':
    is_enable = False
    distance = -1
    u_max = 0.5
    K_p = math.pi / 3

    # initialize node
    rospy.init_node('rover_controller', anonymous=True)

    # publish /cmd_vel to drive rover and
    # initialize its message container
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    msg = Twist()

    rover = Pose2D()
    vertice = Pose2D()
    target = Pose2D()

    # wait until the service point creator to get the next action
    # is ready and get a proxy of it
    rospy.loginfo_once('[rover_controller] is waiting... get_path_from_map')
    rospy.wait_for_service('get_path_from_map')

    srv4GetPath = rospy.ServiceProxy('get_path_from_map', GetPathFromMap)
    srv4switchWps = rospy.ServiceProxy('switch_waypoint', SwitchWaypoint)

    rospy.Service('set_destination', SetDestination, handle_set_destination)
    rospy.Service('enable_motors', SetMotorEnable, handle_enable_motors)

    # subscribe the topic /odometry/filtered to learn local position of the rover
    rospy.Subscriber('/leo_localization/ground_truth_to_pose', Pose2D, update_position, rover)

    rate = rospy.Rate(100)  # 10 Hz

    while not rospy.is_shutdown():

        if is_enable and distance != -1:
            # rover is set to be moved
            if distance < epsilon:  # reached the point
                # switch waypoint
                req4switchWps = SwitchWaypointRequest(True)
                res4switchWps = srv4switchWps(req4switchWps)
                if res4switchWps.is_finished:
                    is_enable = False
                    pub.publish(Twist())
                    rospy.logwarn('[rover] reached...')
                distance = res4switchWps.distance
            else:
                distance = math.sqrt((res4switchWps.waypoint.x - rover.x) ** 2 +
                                     (res4switchWps.waypoint.y - rover.y) ** 2)

                alpha = math.atan2(res4switchWps.waypoint.y - rover.y,
                                   res4switchWps.waypoint.x - rover.x)

                dot_product = (math.cos(alpha) * math.cos(rover.theta) +
                               math.sin(alpha) * math.sin(rover.theta))

                msg.linear.x = dot_product * u_max if dot_product > 0 else 0
                msg.angular.z = (alpha - rover.theta) * K_p
                rospy.logwarn_throttle(
                    0.1, 'x:%2.1f y:%2.1f theta:%2.1f' % (rover.x, rover.y, rover.theta))

                pub.publish(msg)
        rate.sleep()
