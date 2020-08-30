#!/usr/bin/env python

import rospy
from ros_service.srv import NextPoint, NextPointResponse, NextPointRequest
from geometry_msgs.msg import Point

def handleNextPointResponse(msg):
    rospy.loginfo('responsing... /test_service')
    res = Point(3,2,3)
    return res

if __name__ == '__main__':
    rospy.init_node('client', anonymous=True)
    rospy.loginfo_once('requesting... /test_service')
    try:
        rospy.Service('test_service', NextPoint, handleNextPointResponse)
        service_handler = rospy.ServiceProxy('test_service', NextPoint)
        rospy.wait_for_service('/test_service')
        current = Point(1,3,4)
        destin = Point(4,2,3)
        req = NextPointRequest(current, destin)
        response = service_handler(req)

        rospy.loginfo_once('x:%4.2f , y:%4.2f , z:%4.2f' % (response.Next.x, response.Next.y, response.Next.z))
    except rospy.ServiceException as e:
        rospy.logerr_once('wtf: %s' % e)