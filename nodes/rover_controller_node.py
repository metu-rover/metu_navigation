#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
from leo_rover_localization.srv import SetMotorEnable, SetMotorEnableRequest
from leo_rover_localization.srv import SetDestination, SetDestinationRequest


def rover_listener_callback(msg, args):
    motorEnabler, DestSetter = args
    rospy.loginfo('[rover_controller] responding...')
    if msg.data.endswith('_motors'):
        request = SetMotorEnableRequest(msg.data.startswith('enable'))
        response = motorEnabler(request)
        rospy.loginfo(response.response)
    elif msg.data.startswith('set_waypoint_'):
        rospy.loginfo(msg.data[-2:])
        waypoint = rospy.get_param('waypoint_' + msg.data[-2:])
        destination = Pose2D(waypoint['x'], waypoint['y'], waypoint['theta'])
        request = SetDestinationRequest(destination)
        response = DestSetter(request)
        rospy.loginfo(
            'Waypoint is defined and set to be destination' if response.response else 'the waypoint may be undefined or out of the map')
    elif msg.data.startswith('goto'):
        destination = Pose2D(float(msg.data.split()[1]),float(msg.data.split()[2]),0)
        request = SetDestinationRequest(destination)
        response = DestSetter(request)
        rospy.loginfo(
            'Waypoint is defined and set to be destination' if response.response else 'the waypoint may be undefined or out of the map')
    else:
        rospy.logerr('[rover_controller] unknown command')


if __name__ == '__main__':
    rospy.init_node('rover_controller', anonymous=True)

    srv4enMtr = rospy.ServiceProxy('enable_motors', SetMotorEnable)
    srv4setDst = rospy.ServiceProxy('set_destination', SetDestination)

    rospy.wait_for_service('set_destination')
    rospy.loginfo_once('[rover_controller] connected #set_destination @rover_locomotion')
    rospy.wait_for_service('enable_motors')
    rospy.loginfo_once('[rover_controller] connected #enable_motors @rover_locomotion')
    rospy.Subscriber('rover_listener', String,
                     rover_listener_callback, (srv4enMtr, srv4setDst))

    rospy.spin()
