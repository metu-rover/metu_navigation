#!/usr/bin/env python3

import math
import rospy
import MapDesign
from MapDesign import Obstacles
import numpy as np
from collections import defaultdict
from leo_rover_localization.srv import GetPathFromMap
from leo_rover_localization.srv import SwitchWaypoint, SwitchWaypointResponse
from geometry_msgs.msg import Point
import itertools


class Points():

    def __init__(self, coords, Way):
        self.Coord = coords
        self.Roughness = "Infinity"

        check = self.IsItInside(Way.ObstaclesList)
        PairList = self.AddPointsToPairList(Way.ObstaclesList)
        if check == True:
            Way.Create_Ways(PairList, self.Roughness)
        else:
            Way.Create_Ways(PairList)

    def IsItInside(self, GivenObstacleList=[]):
        # The function checks if the point is in the obstacle

        PairList = []
        for obstacle in GivenObstacleList:
            # the point can only be inside of the obstacles which have roughness.Otherwise the point will be inaccessable.
            if not obstacle.Roughness == "Infinity":
                # that is the reason why the function only checks obstacles that have roguhness.
                counter = 0
                for edge in obstacle.Edges:

                    # The function create lines which has available equation from obstacle's edges
                    # By using these lines the function check the point is in the obstacle or not
                    slope, constant = mathobs1.functionsOfEdges(
                        edge)
                    # the value of meanpoint if it is on the edge
                    y = (slope*obstacle.MeanPoint[0])+constant
                    # the value of the point if it is on the edge
                    y2 = (slope*self.Coord[0])+constant
                    # f(x)=mx+a
                    if not slope == 0:
                        try:
                            # x=(f(x)-a)/m
                            x = (obstacle.MeanPoint[1]-constant)/slope
                            x2 = (self.Coord[1]-constant)/slope
                        except ZeroDivisionError:
                            x, x2 = y, y2
                    # The meanpoint's coords are puted to equations of lines,if the value is smaller than meanpoint's originl value,the edge is upper than meanpoint so that
                    # Coords must be below than the edges if it is inside of the obstacle
                    if y <= obstacle.MeanPoint[1] and y2 <= self.Coord[1]:
                        counter += 1
                    elif y >= obstacle.MeanPoint[1] and y2 >= self.Coord[1]:
                        counter += 1
                    elif x <= obstacle.MeanPoint[0] and x2 <= self.Coord[0]:
                        counter += 1
                    elif x >= obstacle.MeanPoint[0] and x2 >= self.Coord[0]:
                        counter += 1
                    # If the point is correct side of edge counter value is increased.
                if counter == len(obstacle.Edges):
                    # if counter's value is equal to number of edges,the point is in obstacle
                    self.Roughness = obstacle.Roughness
                    return True
        else:
            return False

    def AddPointsToPairList(self, GivenObstacleList=[]):
        PairList = []
        for obstacle in GivenObstacleList:
            for index, coord in enumerate(obstacle.Coords):
                # to create ways from the point to obstacles
                PairList.append((self.Coord, coord))
                if not obstacle.Roughness == "Infinity":
                    PairList.append((self.Coord, obstacle.InsideCoords[index]))
                # to create ways inside the obstacle if the point is inside
        return PairList


class Path():

    def __init__(self, ways, waylist):
        self.Ways = ways
        self.Ways.Way_List = waylist
        self.path = []

    def CreatePath(self, Start_Point, End_Points=list):
        value = 0
        path = []
        total_cost = 0
        # the loop is used for can create paths when there is more than two points
        while value < len(End_Points):
            shortest_way, sum_cost = self.DijkstrasAlgorithm(
                Start_Point, End_Points[value], self.Ways)
            for points in shortest_way:
                # in here all paths is append to one list so that it seems like there is one path
                path.append(points)
            total_cost = total_cost+sum_cost
            Start_Point = End_Points[value]
            value += 1
        return path, total_cost

    def DijkstrasAlgorithm(self, Start_Point, End_Point, ways):
        shortest_way, sum_cost = self.CheckObstacles(
            Start_Point, End_Point, ways)
        # if there is no object between start and endpoint there is no path planninng just draw the way by using pythagorous' theorem
        if shortest_way == []:
            # check obstacles function returns empty list when there is obstacle between start and end points so that path planning algorithm is used for create path
            shortest_way, sum_cost = self.graph.CalculatePath(
                self.graph, Start_Point.Coord, End_Point.Coord)
        return shortest_way, sum_cost

    def CheckObstacles(self, startpoint, endpoint, ways):
        # if there is no obstacle between start and end points instead of using path algorithm the path are drawn as start and end point

        if not startpoint.Roughness == endpoint.Roughness:
            return [], 0  # if they have different roughness the path can't calculate correctly so that this function can not use in this case
        path = []
        can_draw = True
        pairs = [startpoint.Coord, endpoint.Coord]
        for obstacle in ways.ObstaclesList:
            for edge in obstacle.Edges:
                # the funciton check if there is an object between start and end by using pair list just as MapDesign
                if mathobs1.doesCollide(pairs, edge):
                    can_draw = False
                    break
        if can_draw == True:
            path = [[pairs[0], pairs[1], mathobs1.lenghtOfLines(
                pairs, roughness=startpoint.Roughness)]]
            return path, path[0][2]
        return [], 0  # if all edges will collide with the edge of points there is an object between points

    def findRoughness(self, Map):
        # After calculate the path correctly the list which include path information must be returned and saved for path tracking algorithm
        index = 0
        while index < len(self.path):
            roughness = 0
            # In here the funciton calculate real lenght of points by using pythagorous' theorem
            lenght = round(mathobs1.lenghtOfLines(
                self.path[index]), 2)
            # If the lenght is different than in path these lenght has rouhgness
            if lenght != round(self.path[index][2], 2):
                # roughnesss is calculated
                roughness = round(self.path[index][2]/lenght, 2)
                # the lenght is changed to real lenght
                self.path[index][2] = lenght
            # the lenght of world equation is calculated by using pixelsize
            realworldlenght = round(lenght*Map.pixelsize, 2)
            # the real lenght is appended to path
            self.path[index].append(realworldlenght)
            self.path[index].append(roughness)  # roughness is appended to path
            index += 1


class Dijkstra():
    def CalculatePath(self, graph, initial, end):
        # shortest paths is a dict of nodes
        # whose value is a tuple of (previous node, roughness)
        shortest_paths = {initial: (None, 0)}
        current_node = initial
        visited = set()
        while current_node != end:
            visited.add(current_node)
            destinations = graph.PossiblePathPoints[current_node]
            roughness_to_current_node = shortest_paths[current_node][1]
            for next_node in destinations:
                roughness = graph.PointRoughnesseses[(
                    current_node, next_node)] + roughness_to_current_node
                if next_node not in shortest_paths:
                    shortest_paths[next_node] = (current_node, roughness)
                else:
                    current_shortest_roughness = shortest_paths[next_node][1]
                    if current_shortest_roughness > roughness:
                        shortest_paths[next_node] = (current_node, roughness)
            next_destinations = {
                node: shortest_paths[node] for node in shortest_paths if node not in visited}
            if not next_destinations:
                return "Route Not Possible", 0
            # next node is the destination with the lowest roughness
            current_node = min(next_destinations,
                               key=lambda k: next_destinations[k][1])

        # Work back through destinations in shortest path
        path_points = []
        sum_cost = 0
        path_cost = []
        while current_node is not None:
            if sum_cost == 0:
                sum_cost = shortest_paths[current_node][1]

            path_points.append(current_node)
            path_cost.append(shortest_paths[current_node][1])
            next_node = shortest_paths[current_node][0]
            current_node = next_node

        # Reverse path
        path_points = path_points[::-1]
        path = []
        path_cost = path_cost[::-1]
        for index, points in enumerate(path_points):
            if index == len(path_points) - 1:
                break
            # calculate every cost of lines insted of calculate just total cost
            point_cost = path_cost[index+1]-path_cost[index]
            path.append([points, path_points[index+1], point_cost])

        return path, sum_cost


class Graph_d(Dijkstra):
    def __init__(self):
        """
        self.edges is a dict of all possible next nodes
        e.g. {'X': ['A', 'B', 'C', 'E'], ...}
        self.roughnesss has all the roughnesss between two nodes,
        with the two nodes as a tuple as the key
        e.g. {('X', 'A'): 7, ('X', 'B'): 2, ...}
        """
        self.PossiblePathPoints = defaultdict(list)
        self.PointRoughnesseses = {}

    def AddEdges(self, from_node, to_node, roguhness):
        # Note: assumes edges are bi-directional
        self.PossiblePathPoints[from_node].append(to_node)
        self.PossiblePathPoints[to_node].append(from_node)

        self.PointRoughnesseses[(from_node, to_node)] = roguhness
        self.PointRoughnesseses[(to_node, from_node)] = roguhness


class MathOperations():

    def lenghtOfLines(self, Coords=[], roughness="Infinity"):
        # It is pythagorous' theorem
        if not roughness == "Infinity":
            return np.sqrt((((Coords[1][1] - Coords[0][1]) ** 2) + ((Coords[1][0] - Coords[0][0]) ** 2)))*roughness
        # If there is roughness, the line length is calculated by multiplying the length roughness.
        else:
            return np.sqrt(((Coords[1][1] - Coords[0][1]) ** 2) + ((Coords[1][0] - Coords[0][0]) ** 2))

    def doesCollide(self, Pairs=[()], Edges=[()]):
        a1 = Pairs[1][1]-Pairs[0][1]
        b1 = Pairs[0][0]-Pairs[1][0]
        c1 = a1*Pairs[0][0] + b1*Pairs[0][1]
        a2 = Edges[1][1]-Edges[0][1]
        b2 = Edges[0][0]-Edges[1][0]
        c2 = a2*Edges[0][0] + b2*Edges[0][1]
        det = a1*b2-a2*b1
        if(det != 0):
            x = (b2*c1 - b1*c2)/det
            y = (a1*c2 - a2*c1)/det
            if(x >= min(Pairs[0][0], Pairs[1][0]) and x <= max(Pairs[0][0], Pairs[1][0]) and x >= min(Edges[0][0], Edges[1][0]) and x <= max(Edges[0][0], Edges[1][0])
                    and y >= min(Pairs[0][1], Pairs[1][1]) and y <= max(Pairs[0][1], Pairs[1][1])
                    and y >= min(Edges[0][1], Edges[1][1]) and y <= max(Edges[0][1], Edges[1][1])):
                return True
            return False

    def functionsOfEdges(self, Edges=[]):
        # f(x)=mx+a
        try:
            m = (Edges[1][1]-Edges[0][1])/(Edges[1][0]-Edges[0][0])
        except ZeroDivisionError:
            # it is prevent the error of int/0 or 0/0 error and return constant value of function
            return 0, Edges[0][1]
        a = Edges[0][1]-(m*Edges[0][0])  # it gives constant value of function
        return m, a


def handle_get_path_from_map(msg):
    global MapNp, WayList, mathobs1
    global current_path, index
    for ways in MapNp["Paths"]:
        WayList.append(ways)

    path = Path(MapDesign.Ways(MapNp["ObstacleList"]), WayList)

    map1 = MapDesign.Map((150, 100), (1920, 1080))

    startPoint = Points((msg.curr.x, msg.curr.y), path.Ways)
    endPoint = Points((msg.dest.x, msg.curr.y), path.Ways)

    path.graph = Graph_d()
    for point in path.Ways.Way_List:  # the possible path points is appended to dijsktra algorithm's graph
        path.graph.AddEdges(point[0], point[1], point[2])
    path.path, sum_cost = path.DijkstrasAlgorithm(
        startPoint, endPoint, path.Ways)

    rospy.loginfo('responsing... /path_planner')

    current=path.path[0]
    if current=="R":
        current_path=path.path[0]
    else:
        a=[[0,0],[0,0],path.path[0][2]]
        a[0][0]=current[0][0]-200
        a[0][1]=current[0][1]-324
        a[1][0]=current[1][0]-200
        a[1][1]=current[1][1]-324
        a[0]=tuple(a[0])
        a[1]=tuple(a[1])
        current_path=a
   
    index = 0
    return True


def handle_switch_waypoints(msg):
    global current_path, index

    if msg.is_next:
        index += 1
    else:
        index -= 1

    if index < len(current_path):
        waypoint = current_path[index]
        return SwitchWaypointResponse(False, Point(waypoint[0], waypoint[1], 0))
    else:
        return SwitchWaypointResponse(True, Point(-1, -1, -1))


if __name__ == "__main__":
    WayList = []
    ObstacleList = []
    mathobs1 = MathOperations()
    current_path = [Point(0, 0, 0)]
    index = 0
    rospy.init_node('path_planner_service', anonymous=True)

    if len(rospy.myargv()) == 1:
        rospy.logerr(
            'usage: rosrun leo_rover_localization service_node.py <path_to_map.npz>')
    else:
        np_load_old = np.load
        # modify the default parameters of np.loadWe
        np.load = lambda *a, **k: np_load_old(*a, allow_pickle=True, **k)
        MapNp = np.load(rospy.myargv()[1])
        # call load_data with allow_pickle implicitly set to true
        # # restore np.load for future normal usage
        np.load = np_load_old

        rospy.Service('get_path_from_map', GetPathFromMap,
                      handle_get_path_from_map)
        
        rospy.Service('get_next_waypoint', SwitchWaypoint,
                      handle_switch_waypoints)

        rospy.loginfo_once('requesting... /point_creator')

        rospy.spin()
