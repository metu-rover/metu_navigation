#!/usr/bin/env python

import math
import rospy
import numpy as np
from collections import defaultdict
from leo-rover-locomotion.srv import GetPathFromMap, PlanPathFromMap
from geometry_msgs.msg import Point
import itertools


class Map():
    def __init__(self, mapsize=tuple, screensize=tuple):
        screenx, screeny, multiply = self.findMapSize(mapsize, screensize)
        # it is for define a map that fit the screen but include as much pixel as could be and correct path length in world
        self.imagesize = (screenx, screeny)
        # every pixel in the image equals to this length in the world
        self.pixelsize = round((1*1.0)/multiply, 3)

    def findMapSize(self, mapsize, screensize):
        screenx, screeny = mapsize[0], mapsize[1]
        multiply = 2  # multiply of zoom
        # if map is smaller than screensize the map is zoomed
        if mapsize[0] < screensize[0] and mapsize[1] < screensize[1]:
            while True:
                screenx = mapsize[0]*multiply
                screeny = mapsize[1]*multiply
                # if zoomed mapsize is bigger than max screensize loop is broken
                if screenx > screensize[0] or screeny > screensize[1]:
                    multiply -= 1  # maximum multiply
                    break
                multiply += 1
            return mapsize[0]*multiply, mapsize[1]*multiply, multiply
        else:  # if map is bigger than screensize the map is zoomed out
            while True:
                screenx = mapsize[0]/multiply
                screeny = mapsize[1]/multiply
                if screenx < 1920 or screeny < 1080:  # if zoomed mapsize is smaller than max screensize loop is broken
                    multiply -= 1
                    break
                multiply += 1
            # to remove float numbers as result of dividing use int function
            return int(mapsize[0]/multiply), int(mapsize[1]/multiply), 1/multiply


class Obstacles():

    def __init__(self, Coords, Roughness="Infinity"):
        global ObstacleList
        self.Coords = Coords  # coordinates must be defined sequentially
        self.InsideCoords = []
        self.Edges = []
        self.Roughness = Roughness
        # to define outside and inside coords
        self.MeanPoint = np.mean(self.Coords, axis=0)

        ObstacleList.append(self)  # it is necessary for ways class

        self.CoordsToEdges()  # define edges by using coords of obstacle
        if not self.Roughness == "Infinity":  # to define ways in the obstacles that can passable
            self.InsideCoords = self.ShiftCoords(number=-1)
            self.InsideCoords.append((self.MeanPoint[0], self.MeanPoint[1]))
        self.Coords = self.ShiftCoords()

    def CoordsToEdges(self):
        index = 0
        while index != len(self.Coords)-1:
            # each coordinate is combined with the next
            self.Edges.append((self.Coords[index], self.Coords[index+1]))
            index += 1
        self.Edges.append((self.Coords[index], self.Coords[0]))

    def ShiftCoords(self, number=1):
        new_coords = []

        # Coords must be shifted for do not collide with any own edges
        for coord in self.Coords:
            if coord[0] > self.MeanPoint[0]:
                new_coords_x = coord[0]+number
            else:
                new_coords_x = coord[0]-number
            if coord[1] > self.MeanPoint[1]:
                new_coords_y = coord[1]+number
            else:
                new_coords_y = coord[1]-number

            new_coords.append((new_coords_x, new_coords_y))
        return new_coords
        # If the x-axis value of the coordinate is greater than the value of the mean point, this coordinate is to the right of the midpoint,
        # so it must be shifted to the right for outside coordinates and must be shifred to the left for inside coordinates.


class Ways():

    def __init__(self, ObstaclesList=[]):
        self.ObstaclesList = ObstaclesList
        self.Pair_List = []  # to take combinations of the all obstacle's all points
        self.Way_List = []

        self.Create_Rough_Pairlist()
        self.Crate_PairList()
        self.Create_Ways(self.Pair_List)

    def Create_Ways(self, GivenPairs=[], Roughness=False):
        for pairs in GivenPairs:  # given pairs include all combinations of all points
            can_draw = True
            for obstacle in self.ObstaclesList:
                for edge in obstacle.Edges:  # if the pairs collide with any edge these pairs are deleted for path planning
                    # else add to possible accesable points
                    if mathobs1.doesCollide(pairs, edge):
                        can_draw = False
                        break
            if can_draw == True:
                if not Roughness == False:
                    self.Way_List.append(
                        (pairs[0], pairs[1], mathobs1.lenghtOfLines(pairs, roughness=Roughness)))
                    # If the obstacle has a roughness, the ways' lenght are added by multiplying their roughness
                    # Becuse of the road roughness the vehicles must pass the area slower
                    # By using these the algorithm will choose most efficient way
                else:
                    self.Way_List.append(
                        (pairs[0], pairs[1], mathobs1.lenghtOfLines(pairs)))
                    # the ways are added to list with their lenght

    def Create_Rough_Pairlist(self):
        # These loops exist for crate a rough pair list which include only Rough coords for doesCollide function

        for obstacle in self.ObstaclesList:
            Rough_Pairlist = []
            if not obstacle.Roughness == "Infinity":
                for pair in obstacle.InsideCoords:
                    # In here the ways are drawn if an obstacle inside of another obstacle
                    for objects in self.ObstaclesList:
                        if not objects.Roughness == "Infinity":
                            for coords in objects.Coords:
                                Rough_Pairlist.append((pair, coords))
                                # until this part we included all combinatons of one obstacle's coord and other obstacle's insidecoord to list
                for inside in itertools.combinations(obstacle.InsideCoords, 2):
                    Rough_Pairlist.append(inside)
                # in that part we included all combinations of one obstacle's coords between themselves to list for increase the ways that inside of obstacle
                for value, coord in enumerate(obstacle.Coords):
                    self.Way_List.append((coord, obstacle.InsideCoords[value], mathobs1.lenghtOfLines(
                        [coord, obstacle.InsideCoords[value]], roughness=obstacle.Roughness)))
                    # In order to the algorithm access to an obstacle which is passable the outside coords and inside coords of obstacle must be connected
                    # in that part we connected  inside coords and coords
                self.Create_Ways(Rough_Pairlist, obstacle.Roughness)
                # After create the pairs who have roughness the list is sent to create ways funciton

    def Crate_PairList(self):
        # The pair list contains all possible lines by the coordinate pairs generated. If these lines do not collide, they can be used as paths.
        PairList = []
        for obstacle in self.ObstaclesList:
            for coords in obstacle.Coords:
                PairList.append(coords)
        # It is for crate the list which contain all coords for take combinations of all of them
        for pair in itertools.combinations(PairList, 2):
            self.Pair_List.append(pair)
            # itertools.combinations function returns all combinations of coordinate pairs of the paired list.


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
    global ObstacleList
    """""""""*************    IDENTIFIED   OBJECTS    *************"""""""""
    # do not define concave shapes,concave shapes must be divided into convex shapes and defined in this way
    # If the user wants to define an obstacle inside another obstacle,s/he must first define an inside one.
    Infinity1 = Obstacles(
        [(270, 202), (287, 258), (356, 316), (307, 399), (229, 416), (172, 349)])
    Infinity2 = Obstacles([(138, 626), (256, 748), (346, 752),
                           (480, 906), (307, 1004), (73, 947), (49, 796)])
    Infinity3 = Obstacles([(402, 708), (358, 570), (463, 404),
                           (640, 209), (795, 191), (833, 241), (857, 385), (783, 461)])
    Infinity4 = Obstacles([(664, 652), (747, 712), (864, 659),
                           (1084, 703), (1118, 823), (1080, 975), (724, 962), (488, 813)])
    Infinity5 = Obstacles(
        [(984, 118), (1093, 95), (1115, 238), (1023, 287), (947, 252)])
    Infinity6 = Obstacles(
        [(1025, 391), (1191, 337), (1296, 361), (1353, 493), (1153, 663), (1025, 579)])

    Rough1 = Obstacles(
        [(897, 423), (926, 495), (986, 461), (975, 418)], Roughness=3)
    Rough2 = Obstacles(
        [(147, 455), (219, 457), (203, 551), (166, 545)], Roughness=4)
    Rough3 = Obstacles([(706, 525), (797, 630), (1017, 617),
                        (997, 397), (908, 365)], Roughness=1.8)
    Rough4 = Obstacles([(85, 503), (124, 424), (278, 451),
                        (250, 596), (132, 578)], Roughness=2)

    """""""""*************    IDENTIFIED   OBJECTS    *************"""""""""
    Way = Ways(ObstacleList)
    path = Path(Way, Way.Way_List)

    map1 = Map((150, 100), (1920, 1080))

    startPoint = Points((msg.curr.x, msg.curr.y), path.Ways)
    endPoint = Points((msg.dest.x, msg.dest.x), path.Ways)

    path.graph = Graph_d()
    for point in path.Ways.Way_List:  # the possible path points is appended to dijsktra algorithm's graph
        path.graph.AddEdges(point[0], point[1], point[2])
    path.path, sum_cost = path.DijkstrasAlgorithm(
        startPoint, endPoint, path.Ways)
    path.findRoughness(map1)

    rospy.loginfo('responsing... /path_planner')

    current_path = path.path[0][1]
    return Point(current_path[0], current_path[1], 0)


if __name__ == "__main__":
    ObstacleList = []
    mathobs1 = MathOperations()

    rospy.init_node('path_planner_service', anonymous=True)
    rospy.loginfo_once('the service /get_path_from_map is ready...')

    rospy.Service('get_path_from_map', GetPathFromMap, handle_get_path_from_map)

    rospy.spin()
