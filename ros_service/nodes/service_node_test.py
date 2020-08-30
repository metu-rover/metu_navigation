#!/usr/bin/env python

import rospy
import itertools
import numpy as np
from random import randint
from ros_service.msg import NextPoint
from geometry_msgs.msg import Point


def visualizer(msg, obs):
    res = Point()
    res.x = randint()
    res.y = randint()
    res.z = randint()
    return res


if __name__ == "__main__":
    ObstacleList = [Point()] * 2

    rospy.init_node('service_node', anonymous=True)

    rospy.spin()
