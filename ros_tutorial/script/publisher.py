#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

if __name__ == '__main__':

    counter = 0
    rospy.init_node('publisher', anonymous=True)

    msg = Twist()

    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10)  # 100 Hz
    while not rospy.is_shutdown():

        if counter < 500:
            msg.linear.x = 0.5
            counter += 1
        elif counter < 700:
            msg.angular.z = 0.5
            msg.linear.x = 0.5
            counter += 1
        else:
            msg.linear.x = 0
            msg.angular.z = 0

        pub.publish(msg)
        rate.sleep()
