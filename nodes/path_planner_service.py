#!/usr/bin/env python3

import math
import rospy
from leo_rover_localization.srv import GetPathFromMap, GetPathFromMapResponse
from leo_rover_localization.srv import GetNextVertex, GetNextVertexResponse
from geometry_msgs.msg import Pose2D
from PathPlanner import Points, path, Graph_d


def handle_get_path_from_map(msg):
    global current_path, index

    startPoint = Points((msg.target.x, msg.target.y), path.Ways)
    endPoint = Points((msg.destin.x, msg.destin.y), path.Ways)

    path.graph = Graph_d()
    for point in path.Ways.Way_List:  # the possible path points is appended to dijsktra algorithm's graph
        path.graph.AddEdges(point[0], point[1], point[2])

    path.path, sum_cost = path.DijkstrasAlgorithm(startPoint, endPoint, path.Ways)

    rospy.loginfo('#get_path_from_map responding...')

    if sum_cost != -1:
        index = 0
        current_path = [msg.target]

        for wp in path.path:
            current_path.append(Pose2D(wp[1][0], wp[1][1], math.atan2(
                wp[1][1] - current_path[-1].y, wp[1][0] - current_path[-1].x)))

        for vertex in current_path:
            rospy.loginfo('(x:%3.2f y:%3.2f)' %
                          (vertex.x, vertex.y))

        return GetPathFromMapResponse(current_path, True)
    else:
        return GetPathFromMapResponse([], False)


def handle_get_next_vertex(msg):
    if msg.to_increment:
        at_boundary = index == len(current_path) - 1
        if not at_boundary:
            index += 1

        distance = math.sqrt((current_path[index].x - current_path[index - 1].x)**2 +
                             (current_path[index].y - current_path[index - 1].y)**2)

        next_vertex = current_path[index]
    else:
        at_boundary = index == 0
        if not at_boundary:
            index -= 1

        distance = math.sqrt((current_path[index].x - current_path[index + 1].x)**2 +
                             (current_path[index].y - current_path[index + 1].y)**2)

        if current_path[index].theta < 0:
            next_vertex = Pose2D(
                current_path[index].x, current_path[index].y, current_path[index].theta + math.pi)
        else:
            next_vertex = Pose2D(
                current_path[index].x, current_path[index].y, current_path[index].theta - math.pi)

    rospy.loginfo('#get_next_vertex responding...')

    return GetNextVertexResponse(at_boundary, distance, next_vertex)


if __name__ == "__main__":
    current_path = [Pose2D(0, 0, 0)]
    index = -1
    rospy.init_node('path_planner', anonymous=True)

    if len(rospy.myargv()) == 1:
        rospy.logerr(
            'usage: rosrun leo_rover_localization path_planner_service.py /path/to/map.npz')
    else:
        rospy.Service('get_path_from_map', GetPathFromMap,
                      handle_get_path_from_map)
        rospy.loginfo_once(
            '#get_path_from_map running @path_planner')

        rospy.Service('get_next_vertex', GetNextVertex,
                      handle_get_next_vertex)
        rospy.loginfo_once(
            '#get_next_vertex running @path_planner')

        rospy.spin()
