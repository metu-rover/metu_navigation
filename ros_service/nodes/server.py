#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from ros_service.srv import NextPoint, NextPointResponse


def handleNextPointResponse(msg):
    return NextPointResponse(1, 2, 3)


if __name__ == "__main__":
    rospy.init_node('server', anonymous=True)

    srv = rospy.Service('test_service', NextPoint, handleNextPointResponse)

    rospy.spin()
