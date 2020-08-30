#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from ros_service.srv import NextPoint, NextPointResponse


def handleNextPointResponse(msg):
    rospy.loginfo('responsing... /test_service')
    res = Point(3,2,3)
    return res


if __name__ == "__main__":
    rospy.init_node('test_service_server', anonymous=True)

    rospy.Service('test_service', NextPoint, handleNextPointResponse)

    rospy.spin()
