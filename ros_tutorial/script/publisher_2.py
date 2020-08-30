#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import Twist, Pose

def callback(msg, pose):
    pose = msg

if __name__ == '__main__':
    rospy.init_node('publisher', anonymous=True)
    msg = Twist()
    pose = Pose()
    pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    rate = rospy.Rate(10)
    counter = 0
    rospy.Subscriber("/turtle1/pose", Pose, callback, pose)
    while not rospy.is_shutdown():
        if counter < 30:
            msg.linear.y = 1
            counter += 1
        elif counter < 45:
            msg.angular.z = -math.pi/2
            msg.linear.y = 0
            msg.linear.x = 1
            counter += 1
        else:
            msg.angular.z = 0
            msg.linear.y = 0
            msg.linear.x = 0

        pub.publish(msg)
        rate.sleep()
