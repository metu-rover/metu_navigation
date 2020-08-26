import cv2
from MapDesign import Obstacles
import numpy as np
import MathAndOperations
import MapDesign
import time
import PlanningAlgorithms
import copy
from collections import defaultdict
start_time = time.time()

np_load_old = np.load
# modify the default parameters of np.loadWe
np.load = lambda *a, **k: np_load_old(*a, allow_pickle=True, **k)
MapNp = np.load("Map_01.npz")
# call load_data with allow_pickle implicitly set to true
# # restore np.load for future normal usage
np.load = np_load_old

WaysList = []

math = MathAndOperations.MathOperations()
""" tahmini waypointlere noktalardan yakın olanlardan götür ve uygunsa geri getir veya bir sonraki noktaya götür """


def main():
    # Class Ways work with list not np arrays actually ı was too lazy to do so class ways work with list
    for ways in MapNp["Paths"]:
        WaysList.append(ways)  # convert a np array to list

    # np save is unpack here
    path = Path(MapDesign.Ways(MapNp["ObstacleList"]), WaysList)
    map1 = MapDesign.Map((150, 100), (1920, 1080))
    endPoints = []  # for multi end points we need an array
    print("Enter the starting point:")
    startPoint = MapDesign.Points((int(input()), int(input())), path.Ways)
    print("Enter the points to visit and after that enter the ending point:")
    while True:
        xaxis = int(input())
        yaxis = int(input())
        points = MapDesign.Points((xaxis, yaxis), path.Ways)
        endPoints.append(points)
        print("If the entering is finished enter 1 else enter 2 after continue to enter coordinates")
        control = int(input())
        if control == 1:
            break
    #path.drawlines(path.Image, path.Ways.Way_List, 0, 0, 255, 1)
    global start_time
    # for calculate the working time correctly i start the time after entering
    start_time = time.time()
    path.graph = PlanningAlgorithms.Graph_d()
    for point in path.Ways.Way_List:  # the possible path points is appended to dijsktra algorithm's graph
        path.graph.AddEdges(point[0], point[1], point[2])
    path.path, sum_cost = path.CreatePath(startPoint, endPoints)
    """if not path.path=="Route Not Possible":
        print(path.path)
        path.drawlines(path.Image,path.path,255,0,0,4)
    else:                                                   #If there is no way to access start to end.The funciton draw the points.
        cv2.circle(path.Image,startPoint.Coord,4,(0,255,255),-1) 
        for k in endPoints:
            cv2.circle(path.Image,endPoints[k].Coord,4,(0,255,255),-1)  
    cv2.imshow("image",path.Image)
    """
    path.findRoughness(map1)
    print("path", path.path, "path cost", sum_cost)
    np.savez("deneme", path_points=path.path)  # It is saved for path tracking
    print("--- %s seconds ---" % (time.time() - start_time))

    cv2.waitKey(0)
    cv2.destroyAllWindows()


class Path():

    def __init__(self, ways, waylist):

        self.Image = cv2.imread("map_01.png")
        # eğer resimi kullanmak isterseniz aşşağıdaki 2 satırı yorum satırı yapıp üsttekini açın
        """ self.Image=np.zeros((map1.screensize[1],map1.screensize[0],3),np.uint8)
        self.Image[:]=[255,255,255] """
        self.Ways = ways
        self.Ways.Way_List = waylist
        self.path = []

    def drawlines(self, image, GivenLines=[], Blue=0, Green=0, Red=0, Thickness=2):
        for line in GivenLines:
            start = (int(line[0][0]), int(line[0][1]))
            # cv2 function only work with integers that is why some float values must converted
            end = (int(line[1][0]), int(line[1][1]))
            # it draws lines which is given by the user
            cv2.line(image, start, end, (Blue, Green, Red), Thickness)
            # it draws circle to the lines start and end points
            cv2.circle(image, start, 1, (0, 255, 255), -1)
            cv2.circle(image, end, 1, (0, 255, 255), -1)

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
        global math
        if not startpoint.Roughness == endpoint.Roughness:
            return [], 0  # if they have different roughness the path can't calculate correctly so that this function can not use in this case
        path = []
        can_draw = True
        pairs = [startpoint.Coord, endpoint.Coord]
        for obstacle in ways.ObstaclesList:
            for edge in obstacle.Edges:
                # the funciton check if there is an object between start and end by using pair list just as MapDesign
                if math.doesCollide(pairs, edge):
                    can_draw = False
                    break
        if can_draw == True:
            path = [[pairs[0], pairs[1], math.lenghtOfLines(
                pairs, roughness=startpoint.Roughness)]]
            return path, path[0][2]
        return [], 0  # if all edges will collide with the edge of points there is an object between points

    def findRoughness(self, Map):
        # After calculate the path correctly the list which include path information must be returned and saved for path tracking algorithm
        index = 0
        while index < len(self.path):
            roughness = 0
            # In here the funciton calculate real lenght of points by using pythagorous' theorem
            lenght = round(math.lenghtOfLines(self.path[index]), 2)
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


if __name__ == "__main__":
    main()
