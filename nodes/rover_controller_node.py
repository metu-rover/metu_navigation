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
        pub.publish(Twist())
        return "Self-control is disabled. You are boss now!"


if __name__ == '__main__':
    is_enable = False
    to_disable = False
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
    vertex = Pose2D()

    # wait until the service point creator to get the next action
    # is ready and get a proxy of it
    rospy.loginfo_once('[rover_controller] waiting for get_path_from_map')
    rospy.wait_for_service('get_path_from_map')

    srv4GetPath = rospy.ServiceProxy('get_path_from_map', GetPathFromMap)
    srv4NextVertex = rospy.ServiceProxy('get_next_vertex', GetNextVertex)

    rospy.Service('set_destination', SetDestination, handle_set_destination)
    rospy.Service('enable_motors', SetMotorEnable, handle_enable_motors)

    # subscribe the topic /odometry/filtered to learn local position of the rover
    rospy.Subscriber('/leo_localization/ground_truth_to_pose2D', Pose2D, update_position, rover)

    rate = rospy.Rate(100)  # 10 Hz

    while not rospy.is_shutdown():

        if is_enable and distance != -1:
            # rover is set to be moved
            if distance < epsilon:  # reached the point
                # switch waypoint
                if to_disable:
                    is_enable = False
                    pub.publish(Twist())
                    rospy.logwarn('[rover_controller] Self-control is disabled due to arrival...')
                    to_disable = False
                else:
                    req4NextVertex = GetNextVertexRequest(True)
                    res4NextVertex = srv4NextVertex(req4NextVertex)
                    distance = res4NextVertex.distance
                    vertex = res4NextVertex.next_vertex
                    to_disable = res4NextVertex.at_boundary
            else:
                distance = math.sqrt((res4NextVertex.next_vertex.x - rover.x) ** 2 +
                                     (res4NextVertex.next_vertex.y - rover.y) ** 2)

                alpha = math.atan2(res4NextVertex.next_vertex.y - rover.y,
                                   res4NextVertex.next_vertex.x - rover.x)

                dot_product = (math.cos(alpha) * math.cos(rover.theta) +
                               math.sin(alpha) * math.sin(rover.theta))

                msg.linear.x = dot_product * u_max if dot_product > 0 else 0
                msg.angular.z = (alpha - rover.theta) * K_p

                pub.publish(msg)
        rate.sleep()
