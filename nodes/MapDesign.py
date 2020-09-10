#!/usr/bin/env python3

import numpy as np
import itertools
import rospy

class MathOperations():

    def lenghtOfLines(self, Coords=[], roughness="Infinity"):
        # It is pythagorous' theorem
        if not roughness == "Infinity":
            return np.sqrt((((Coords[1][1] - Coords[0][1]) ** 2) + ((Coords[1][0] - Coords[0][0]) ** 2)))*roughness
        # If there is roughness, the line length is calculated by multiplying the length roughness.
        else:
            return np.sqrt(((Coords[1][1] - Coords[0][1]) ** 2) + ((Coords[1][0] - Coords[0][0]) ** 2))

    def doesCollide(self, Pairs=[()], Edges=[()]):
        # Original form of the function is under the all functions
        # Does collide function is used for check two lines if they collide or not
        if ((Edges[1][1] - Edges[0][1]) * (Pairs[1][0] - Pairs[0][0]) - (Edges[1][0] - Edges[0][0]) * (
                Pairs[1][1] - Pairs[0][1])) == 0:
            return False
        uA = ((Edges[1][0] - Edges[0][0]) * (Pairs[0][1] - Edges[0][1]) - (Edges[1][1] - Edges[0][1]) * (
            Pairs[0][0] - Edges[0][0])) / (
            (Edges[1][1] - Edges[0][1]) * (Pairs[1][0] - Pairs[0][0]) - (Edges[1][0] - Edges[0][0]) * (
                Pairs[1][1] - Pairs[0][1]))
        uB = ((Pairs[1][0] - Pairs[0][0]) * (Pairs[0][1] - Edges[0][1]) - (Pairs[1][1] - Pairs[0][1]) * (
            Pairs[0][0] - Edges[0][0])) / (
            (Edges[1][1] - Edges[0][1]) * (Pairs[1][0] - Pairs[0][0]) - (Edges[1][0] - Edges[0][0]) * (
                Pairs[1][1] - Pairs[0][1]))
        if (uA >= 0 and uA <= 1 and uB >= 0 and uB <= 1):
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


class Map():
    def __init__(self, mapsizeM2=tuple, screensize=tuple):
        screenx, screeny, multiply = self.findMapSize(mapsizeM2, screensize)
        self.multi = multiply
        # it is for define a map that fit the screen but include as much pixel as could be and correct path length in world
        self.mapsizepixel = (screenx, screeny)
        # every pixel in the image equals to this length in the world
        self.pixelsize = round(1/multiply, 3)

    def findMapSize(self, mapsize, screensize):
        screenx, screeny = mapsize[0], mapsize[1]
        multiply = 2  # multiply of zoom
        tryfloatnumbers = 0
        # if map is smaller than screensize the map is zoomed
        if mapsize[0] < screensize[0] and mapsize[1] < screensize[1]:
            while True:
                screenx = mapsize[0]*multiply
                screeny = mapsize[1]*multiply
                # if zoomed mapsize is bigger than max screensize loop is broken
                if screenx > screensize[0] or screeny > screensize[1]:
                    if tryfloatnumbers == 1:
                        break
                    multiply -= 1  # maximum multiply
                    tryfloatnumbers = 1
                if tryfloatnumbers == 1:
                    multiply += 0.1
                else:
                    multiply += 1
            return round(mapsize[0]*multiply, 2), round(mapsize[1]*multiply, 2), multiply
        else:
            # if map is bigger than screensize the map is zoomed out

            while True:
                screenx = mapsize[0]/multiply
                screeny = mapsize[1]/multiply
                # if zoomed mapsize is smaller than max screensize loop is broken
                if screenx < screensize[0] or screeny < screensize[1]:
                    if tryfloatnumbers == 1:
                        break
                    multiply -= 1  # maximum multiply
                    tryfloatnumbers = 1
                if tryfloatnumbers == 1:
                    multiply += 0.1
                else:
                    multiply += 1
            # to remove float numbers as result of dividing use int function
            return round(mapsize[0]/multiply, 2), round(mapsize[1]/multiply, 2), 1/multiply

    def createObstacles(self, obstaclesCoords, Roughness="Infinity"):
        k = 0
        while k <= len(obstaclesCoords[Roughness])-1:
            optimized = []
            if Roughness == "Rough":
                coords = obstaclesCoords[Roughness][k]
            if Roughness == "Infinity":
                coords = (obstaclesCoords[Roughness][k], "Infinity")
            for values in coords[0]:
                optimized.append((values[0]*map1.multi, values[1]*map1.multi))
            optimized = tuple(optimized)
            obstacle = Obstacles(optimized, Roughness=coords[1])
            obstacle.sequentalFsunctions()
            k += 1


class Obstacles():

    def __init__(self, Coords, Roughness="Infinity"):
        global ObstacleList
        self.Coords = Coords  # coordinates must be defined sequentially
        self.InsideCoords = []
        self.Edges = []
        self.Roughness = Roughness
        # to define outside and inside coords
        self.MeanPoint = np.mean(self.Coords, axis=0)

    def CoordsToEdges(self):
        index = 0
        while index != len(self.Coords)-1:
            # each coordinate is combined with the next
            self.Edges.append((self.Coords[index], self.Coords[index+1]))
            index += 1
        self.Edges.append((self.Coords[index], self.Coords[0]))

    def ShiftCoords(self, number=1):
        new_coords = []
        outsquare = map1.mapsizepixel
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
            if not new_coords_x <= 0 and not new_coords_x >= outsquare[0] and not new_coords_y <= 0 and not new_coords_y >= outsquare[1]:
                new_coords.append((new_coords_x, new_coords_y))
        return new_coords
        # If the x-axis value of the coordinate is greater than the value of the mean point, this coordinate is to the right of the midpoint,
        # so it must be shifted to the right for outside coordinates and must be shifred to the left for inside coordinates.

    def sequentalFsunctions(self):
        ObstacleList.append(self)  # it is necessary for ways class

        self.CoordsToEdges()  # define edges by using coords of obstacle
        if not self.Roughness == "Infinity":  # to define ways in the obstacles that can passable
            self.InsideCoords = self.ShiftCoords(number=-1)
            self.InsideCoords.append((self.MeanPoint[0], self.MeanPoint[1]))
        if not self.Roughness == "Infinity":
            self.Coords = self.ShiftCoords(number=5)
        else:
            self.Coords = self.ShiftCoords(number=20)


class Ways():

    def __init__(self, ObstaclesList=[]):
        self.ObstaclesList = ObstaclesList
        self.Pair_List = []  # to take combinations of the all obstacle's all points
        self.Way_List = []

    def Create_Ways(self, GivenPairs=[], Roughness=False):
        global math
        for pairs in GivenPairs:  # given pairs include all combinations of all points
            can_draw = True
            for obstacle in self.ObstaclesList:
                for edge in obstacle.Edges:  # if the pairs collide with any edge these pairs are deleted for path planning
                    # else add to possible accesable points
                    if math.doesCollide(pairs, edge):
                        can_draw = False
                        break
            if can_draw == True:
                if not Roughness == False:
                    self.Way_List.append(
                        (pairs[0], pairs[1], math.lenghtOfLines(pairs, roughness=Roughness)))
                    # If the obstacle has a roughness, the ways' lenght are added by multiplying their roughness
                    # Becuse of the road roughness the vehicles must pass the area slower
                    # By using these the algorithm will choose most efficient way
                else:
                    self.Way_List.append(
                        (pairs[0], pairs[1], math.lenghtOfLines(pairs)))
                    # the ways are added to list with their lenght

    def Create_Rough_Pairlist(self):
        # These loops exist for crate a rough pair list which include only Rough coords for doesCollide function
        global math
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
                    self.Way_List.append((coord, obstacle.InsideCoords[value], math.lenghtOfLines(
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
        global math

        PairList = []
        for obstacle in GivenObstacleList:
            # the point can only be inside of the obstacles which have roughness.Otherwise the point will be inaccessable.
            if not obstacle.Roughness == "Infinity":
                # that is the reason why the function only checks obstacles that have roguhness.
                counter = 0
                for edge in obstacle.Edges:

                    # The function create lines which has available equation from obstacle's edges
                    # By using these lines the function check the point is in the obstacle or not
                    slope, constant = math.functionsOfEdges(edge)
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


ObstacleList = []
math = MathOperations()
map1 = Map((36, 43), (714, 850))


def main(path_to_map):
    global ObstacleList

    """""""""*************    IDENTIFIED   OBJECTS    *************"""""""""
    # do not define concave shapes,concave shapes must be divided into convex shapes and defined in this way
    # If the user wants to define an obstacle inside another obstacle,s/he must first define an inside one.
    obstaclesCoords = {"Infinity": (((34.95614035087718, 37.67543859649121), (29.51754385964911, 42.0175438596491), (28.289473684210513, 42.41228070175436), (24.69298245614034, 42.54385964912279), (25.52631578947367, 39.56140350877191), (34.73684210526314, 35.30701754385963)), ((13.28947368421052, 42.587719298245595), (13.11403508771929, 40.65789473684209), (13.684210526315782, 38.42105263157893), (17.456140350877185, 37.67543859649121), (23.33333333333332, 39.42982456140349), (23.684210526315777, 40.921052631578924), (22.236842105263147, 42.80701754385963)), ((28.991228070175424, 33.46491228070174), (31.359649122807003, 32.543859649122794), (33.28947368421051, 33.11403508771928), (31.271929824561386, 35.48245614035086), (24.51754385964911, 35.35087719298244), (23.157894736842092, 33.50877192982455)), ((33.640350877192965, 33.2017543859649), (33.81578947368419, 29.035087719298232), (28.245614035087705, 25.614035087719284), (23.640350877192972, 27.236842105263143), (22.280701754385955, 32.10526315789472), (25.043859649122794, 31.84210526315788), (26.491228070175424, 29.999999999999986), (27.89473684210525, 29.035087719298232), (29.254385964912267, 28.947368421052616), (30.131578947368407, 29.736842105263143), (30.92105263157893, 31.22807017543858), (31.359649122807003, 32.543859649122794)), ((31.622807017543845, 15.657894736842097), (27.85087719298244, 20.394736842105253), (23.33333333333332, 21.315789473684198), (23.421052631578934, 16.3157894736842), (29.385964912280688, 11.929824561403503)), ((35.96491228070174, 21.710526315789462), (32.45614035087718, 11.842105263157888), (33.333333333333314, 8.771929824561399), (32.543859649122794, 5.26315789473684), (35.96491228070174, 2.63157894736842)), ((32.06140350877192, 24.166666666666654), (31.71052631578946, 23.464912280701743), (30.21929824561402, 24.342105263157883), (30.78947368421051, 24.95614035087718)), ((21.535087719298236, 13.245614035087712), (18.596491228070168, 15.219298245614027), (16.9298245614035, 14.60526315789473), (16.096491228070168, 12.763157894736835), (15.61403508771929, 8.46491228070175), (13.59649122807017, 14.342105263157888), (14.73684210526315, 17.456140350877185), (18.903508771929815, 17.93859649122806), (22.19298245614034, 15.087719298245606)), ((19.078947368421044, 25.52631578947367), (17.10526315789473, 21.535087719298236), (15.570175438596484, 24.912280701754373), (14.649122807017537, 26.885964912280688), (16.973684210526308, 27.807017543859637)), ((12.675438596491222, 28.289473684210513), (13.728070175438589, 32.80701754385963), (16.622807017543852, 29.254385964912267)), ((17.10526315789473, 21.535087719298236), (15.789473684210519, 19.692982456140342), (13.771929824561397, 18.68421052631578), (11.710526315789467, 18.11403508771929), (9.473684210526311, 18.640350877192972), (13.684210526315782, 21.44736842105262), (14.824561403508765, 22.280701754385955), (15.92105263157894, 23.421052631578934)), ((9.473684210526311, 18.640350877192972), (7.1929824561403475, 19.956140350877185), (5.614035087719295, 22.49999999999999), (5.175438596491225, 25.83333333333332), (6.929824561403505, 29.078947368421037), (8.1578947368421, 28.245614035087705), (7.982456140350873, 25.482456140350866), (8.289473684210522, 22.850877192982445), (8.947368421052627, 22.105263157894726), (11.842105263157888, 20.350877192982445)), ((11.359649122807012, 29.605263157894722), (8.245614035087716, 28.289473684210513), (6.929824561403505, 29.078947368421037), (11.798245614035082, 31.359649122807003)), ((13.903508771929818, 4.6052631578947345), (10.35087719298245, 11.271929824561397), (6.622807017543856, 13.157894736842099), (5.614035087719295, 10.570175438596486), (12.105263157894731, 0.0), (13.903508771929818, 0.0)), ((16.710526315789465, 6.5789473684210495), (15.307017543859642, 6.403508771929821), (17.763157894736832, 0.0), (20.87719298245613, 0.0)), ((5.614035087719295, 10.657894736842099), (0.0, 10.657894736842099), (0.0, 0.0), (12.061403508771924, 0.0)), ((8.41, 18.1), (9.61, 18.1), (9.61, 16.9), (8.41, 16.9)), ((19.95, 21.69), (21.150000000000002, 21.69), (21.150000000000002, 20.49), (19.95, 20.49)), ((14.32, 31.71), (15.52, 31.71), (15.52, 30.509999999999998), (14.32, 30.509999999999998)), ((22.06, 14.740000000000002), (23.26, 14.740000000000002), (23.26, 13.54), (22.06, 13.54)), ((21.499999999999996, 37.51), (22.7, 37.51), (22.7, 36.31), (21.499999999999996, 36.31)), ((15.87, 11.21), (17.07, 11.21), (17.07, 10.009999999999998), (15.87, 10.009999999999998)), ((31.02, 6.660000000000004), (32.22, 6.660000000000004), (32.22, 5.460000000000001), (31.02, 5.460000000000001)), ((33.89, 24.89), (35.09, 24.89), (35.09, 23.689999999999998), (33.89, 23.689999999999998)), ((3.14, 30.119999999999997), (4.34, 30.119999999999997), (4.34, 28.92), (3.14, 28.92)), ((8.73, 4.859999999999999), (9.93, 4.859999999999999), (9.93, 3.6599999999999966), (8.73, 3.6599999999999966)), ((5.37688442211055, 1.50753768844221), (6.180904522613061, 1.7587939698492452), (5.728643216080398, 3.216080402010048), (5.2261306532663285, 3.3165829145728623)), ((24.221105527638176, 1.3567839195979892), (20.60301507537687, 7.386934673366829), (21.25628140703516, 9.447236180904516), (27.386934673366817, 10.804020100502505), (30.954773869346713, 5.628140703517584), (29.648241206030132, 4.371859296482409), (27.236180904522595, 6.331658291457282), (25.628140703517573, 5.628140703517584)), ((12.06030150753768, 35.628140703517566), (12.06030150753768, 36.23115577889445), (11.457286432160796, 36.38190954773867), (11.256281407035168, 35.57788944723616)), ((13.366834170854263, 18.643216080402), (13.91959798994974, 18.09045226130652), (16.030150753768833, 19.145728643216067), (15.728643216080393, 19.698492462311545)), ((12.914572864321599, 21.658291457286417), (14.824120603015066, 23.015075376884408), (13.467336683417077, 25.728643216080386), (12.613065326633158, 24.02010050251255)), ((8.89447236180904, 34.37185929648239), (8.743718592964818, 33.7688442211055), (7.788944723618085, 33.66834170854269), (7.688442211055271, 34.72361809045224)), ((8.592964824120598, 0.0), (8.592964824120598, 1.50753768844221), (9.597989949748738, 1.50753768844221), (9.597989949748738, 0.0)), ((11.155778894472355, 33.91959798994973), (11.055276381909541, 33.115577889447216), (11.809045226130646, 33.115577889447216), (11.708542713567832, 33.81909547738691)), ((12.613065326633158, 42.96482412060299), (11.708542713567832, 37.085427135678366), (6.482412060301503, 34.87437185929646), (2.110552763819094, 35.02512562814068), (1.7587939698492452, 39.29648241206028), (4.321608040201002, 39.698492462311535), (5.929648241206026, 41.40703517587937), (10.804020100502505, 42.96482412060299))),
                       "Rough": ((((19.29824561403508, 37.192982456140335), (13.815789473684204, 38.333333333333314), (15.394736842105255, 36.359649122807), (18.991228070175428, 34.60526315789472)), 2.5), (((2.964824120603013, 23.417085427135664), (4.271356783919595, 24.32160804020099), (5.1256281407035145, 28.54271356783918), (4.271356783919595, 32.412060301507516), (3.4170854271356763, 32.86432160804018), (1.7085427135678382, 27.587939698492445)), 1.1), (((4.221105527638188, 33.81909547738691), (5.427135678391957, 32.81407035175877), (5.728643216080398, 33.51758793969847), (5.075376884422107, 34.37185929648239)), 1.2), (((14.723618090452252, 7.53768844221105), (13.91959798994974, 8.140703517587934), (14.020100502512554, 9.597989949748738), (14.92462311557788, 9.296482412060296)), 1.1), (((34.87437185929646, 21.708542713567827), (33.969849246231135, 33.21608040201003), (35.82914572864319, 33.81909547738691), (35.778894472361785, 21.708542713567827)), 1.3), (((17.386934673366824, 13.115577889447227), (19.195979899497477, 13.567839195979891), (19.698492462311545, 12.713567839195973), (19.246231155778883, 11.306532663316576), (18.341708542713555, 10.90452261306532)), 1.08), (((0.603015075376884, 14.723618090452252), (2.7135678391959783, 13.517587939698483), (4.371859296482409, 16.834170854271346), (4.221105527638188, 20.65326633165828), (2.211055276381908, 21.30653266331657), (1.407035175879396, 18.44221105527637)), 1.1))}

    # Infinity: (((Coordsx,Coordsy).......(Coordsx,Coordsy)))
    # Rough : (((Coordsx,Coordsy),Roughness=Float).......((Coordsx,Coordsy),Roughness=Float))
    """""""""*************    IDENTIFIED   OBJECTS    *************"""""""""
    Map.createObstacles(Map, obstaclesCoords)
    Map.createObstacles(Map, obstaclesCoords, "Rough")

    Way = Ways(ObstacleList)

    Way.Create_Rough_Pairlist()
    Way.Crate_PairList()
    Way.Create_Ways(Way.Pair_List)
    np.savez(path_to_map, ObstacleList=ObstacleList, Paths=Way.Way_List)
    # with np.savez the map and obstacles define one time and until the user wants changed something in map program doesn't have to use MapDesign.
    # To sum up it increases program performance.


if __name__ == '__main__':
    rospy.init_node('map_design', anonymous=True)

    if (len(rospy.myargv()) == 1):
        rospy.logerr_once('usage: rosrun leo_rover_localization MapDesign.py /path/to/map.npz')
    else:
        main(rospy.myargv()[1])
