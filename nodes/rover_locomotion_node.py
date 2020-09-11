#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist, Pose, PoseStamped, Pose2D, TransformStamped
from leo_rover_localization.srv import GetNextVertex, GetNextVertexRequest
from leo_rover_localization.srv import GetPathFromMap, GetPathFromMapRequest
from leo_rover_localization.srv import SetDestination
from leo_rover_localization.srv import SetMotorEnable

epsilon = 0.25
epsilon_normal = 3.00


def normal_length(rover, vertex, marker):
    m = (vertex.y - rover.y) / (vertex.x - rover.x)
    return (m * marker.x - marker.y - m * rover.x + rover.y) / math.sqrt(m ** 2 + 1)


def distance_between(pose1, pose2):
    return math.sqrt((pose1.x - pose2.x) ** 2 + (pose1.y - pose2.y) ** 2)


def UpdateYawOf(pose, flag):
    if flag:
        pose.theta += math.pi / 2
    else:
        pose.theta -= math.pi / 2

    return pose


def update_position(msg, rover):
    rover.x = msg.x
    rover.y = msg.y
    rover.theta = msg.theta


def handle_set_destination(msg):
    global rover, distance, destination
    req4GetPath = GetPathFromMapRequest(rover, msg.destination)
    res4GetPath = srv4GetPath(req4GetPath)

    if res4GetPath.is_path_updated:
        global to_disable
        destination = msg.destination
        to_disable = False
        distance = 0

    return res4GetPath.is_path_updated


def handle_enable_motors(msg):
    global is_enable
    global pub
    is_enable = msg.enable

    if is_enable:
        return "Self-control is enabled, Don't panic!"
    else:
        pub.publish(Twist())
        return "Self-control is disabled. You are boss now!"


def handle_base_link_transform(msg):
    global any_markers, edge_markers, marker, destination, rover
    request = GetPathFromMapRequest(rover, destination)
    response = srv4GetPath(request)
    any_markers = False

    if response.is_path_updated:
        global to_disable, distance
        to_disable = False
        distance = 0

    try:
        edge_markers.remove(marker)
        rospy.loginfo('marker detected, position updated')
    except ValueError as e:
        rospy.logerr('marker cannot find in markers')

    return response.is_path_updated


if __name__ == '__main__':
    is_enable = False
    to_disable = False
    any_markers = False
    distance = -1
    u_max = 0.5
    K_p = math.pi / 3

    # initialize node
    rospy.init_node('rover_locomotion', anonymous=True)

    # publish /cmd_vel to drive rover and
    # initialize its message container
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    msg = Twist()

    rover = Pose2D()
    vertex = Pose2D()
    destination = Pose2D()

    # wait until the service get_path_from_map to get a path to go destination vertex
    srv4GetPath = rospy.ServiceProxy('get_path_from_map', GetPathFromMap)
    rospy.loginfo_once('[rover_locomotion] waiting for #get_path_from_map')
    rospy.wait_for_service('get_path_from_map')

    srv4NextVertex = rospy.ServiceProxy('get_next_vertex', GetNextVertex)

    rospy.Service('set_destination', SetDestination, handle_set_destination)
    rospy.loginfo_once('#set_destination running @rover_locomotion')

    rospy.Service('enable_motors', SetMotorEnable, handle_enable_motors)
    rospy.loginfo_once('#enable_motors running @rover_locomotion')

    # subscribe the topic /odometry/filtered to learn local position of the rover
    rospy.Subscriber('/leo_localization/ground_truth_to_pose2D',
                     Pose2D, update_position, rover)

    rospy.Subscriber('/handle_base_link_transform', TransformStamped,handle_base_link_transform)

    rate = rospy.Rate(100)  # 10 Hz

    markers = [Pose2D(float(marker[1]['x']), float(marker[1]['y']), 0)
               for marker in rospy.get_param('tf_static').items()]
    edge_markers = []

    rospy.logdebug_once('rover_locomotion READY!!!')
    while not rospy.is_shutdown():

        if is_enable and distance != -1:
            # rover is set to be moved
            if distance < epsilon:  # reached the point
                # switch waypoint
                if to_disable:
                    is_enable = False
                    pub.publish(Twist())
                    rospy.logwarn(
                        '[rover_locomotion] Self-control is disabled due to arrival...')
                else:
                    req4NextVertex = GetNextVertexRequest(True)
                    res4NextVertex = srv4NextVertex(req4NextVertex)
                    distance = res4NextVertex.distance
                    vertex = res4NextVertex.next_vertex
                    to_disable = res4NextVertex.at_boundary

                    edge_markers = [marker for marker in markers if abs(
                        normal_length(rover, vertex, marker)) < epsilon_normal and not distance_between(marker, rover) < epsilon_normal]
            else:

                distance = distance_between(res4NextVertex.next_vertex, rover)

                if any_markers:
                    rospy.loginfo_throttle(0.5, 'in if any_markers')
                    dot_product = 0
                    alpha = math.atan2(marker.y - rover.y, marker.x - rover.x)

                    
                    if abs(alpha - rover.theta) < epsilon:
                        rospy.loginfo('marker has a margin of 15 deg. at most')
                        any_markers = False
                        try:
                            edge_markers.remove(marker)
                            rospy.loginfo('failed to detect, position did not changed')
                        except ValueError as e:
                            rospy.logerr('marker cannot find in markers')
                    elif abs(alpha - rover.theta) < 2 * epsilon:
                        K_c = K_p / 7
                        rospy.loginfo_throttle(1,'marker has a margin of 30 deg. at most')
                    elif abs(alpha - rover.theta) < 3 * epsilon:
                        K_c = K_p / 5
                        rospy.loginfo_throttle(1,'marker has a margin of 45 deg. at most')
                    else:
                        K_c = K_p / 3
                        rospy.loginfo_throttle(1,'marker has a margin of 90 deg. at most')
                else:
                    # any_marker = True
                    marker_distances = [
                        (distance_between(marker, rover), marker) for marker in edge_markers]
                    marker_distances.sort()

                    if marker_distances != [] and marker_distances[0][0] < epsilon_normal:
                        any_markers = True
                        marker = marker_distances[0][1]
                        rospy.loginfo('enemy spotted!')

                    alpha = math.atan2(res4NextVertex.next_vertex.y - rover.y,
                                       res4NextVertex.next_vertex.x - rover.x)

                    dot_product = (math.cos(alpha) * math.cos(rover.theta) +
                                   math.sin(alpha) * math.sin(rover.theta))

                    K_c = K_p

                msg.linear.x = dot_product * u_max if dot_product >= 0 else 0
                msg.angular.z = (alpha - rover.theta) * K_c

                pub.publish(msg)
        rate.sleep()
