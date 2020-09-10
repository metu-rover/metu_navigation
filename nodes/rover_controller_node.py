#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
from leo_rover_localization.srv import SetMotorEnable, SetMotorEnableRequest
from leo_rover_localization.srv import SetDestination, SetDestinationRequest
from leo_rover_localization.srv import SetReferancePose, SetReferencePoseRequest


def rover_listener_callback(msg, args):
    set_motors, set_destination, set_reference = args
    rospy.loginfo('[rover_controller] responding...')
    if msg.data.endswith('motors'):
        request = SetMotorEnableRequest(msg.data.startswith('enable'))
        response = set_motors(request)
        rospy.loginfo(response.response)
    elif msg.data.startswith('set_waypoint'):
        rospy.loginfo(msg.data[-2:])
        waypoint = rospy.get_param('waypoint_' + msg.data[-2:])
        destination = Pose2D(waypoint['x'], waypoint['y'], waypoint['theta'])
        request = SetDestinationRequest(destination)
        response = set_destination(request)
        rospy.loginfo(
            ('Rover now is directed to x:%2.1f y:%2.1f' % (destination.x, destination.y)) if response.response else 'the waypoint may be undefined or out of the map')
    elif msg.data.startswith('set_destination'):
        destination = Pose2D(float(msg.data.split()[1]), float(msg.data.split()[2]), 0)
        request = SetDestinationRequest(destination)
        response = set_destination(request)
        rospy.loginfo(
            ('Rover now is directed to x:%2.1f y:%2.1f' % (destination.x, destination.y)) if response.response else 'the destination may be undefined or out of the map')
    elif msg.data.startswith('set_pose'):
        position = Pose2D(float(msg.data.split()[1]), float(msg.data.split()[2]), 0)
        request = SetReferencePoseRequest(position)
        response = set_reference(request)
        rospy.loginfo(('Rover now is located at x:%2.1f y:%2.1f' % (
            position.x, position.y)) if response.response else 'something went wrong')
    else:
        rospy.logerr('[rover_controller] unknown command')


if __name__ == '__main__':
    rospy.init_node('rover_controller', anonymous=True)

    set_destination = rospy.ServiceProxy('set_destination', SetDestination)
    enable_motors = rospy.ServiceProxy('enable_motors', SetMotorEnable)
    set_pose = rospy.ServiceProxy('/leo_localization/taring_the_balance', SetReferancePose)

    rospy.wait_for_service('set_destination')
    rospy.loginfo_once('[rover_controller] connected #set_destination @rover_locomotion')

    rospy.wait_for_service('enable_motors')
    rospy.loginfo_once('[rover_controller] connected #enable_motors @rover_locomotion')
    
    rospy.wait_for_service('/leo_localization/taring_the_balance')
    rospy.loginfo_once('[rover_controller] connected #taring_the_balance @leo_localization')
    

    rospy.Subscriber('rover_listener', String,
                     rover_listener_callback, (enable_motors, set_destination, set_pose))

    rospy.spin()
