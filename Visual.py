import cv2
from Map import Obstacles
import numpy as np 
import MathAndOperations
import Map
import time
import PlanningAlgorithms
import copy
start_time = time.time()


np_load_old = np.load
# modify the default parameters of np.loadWe
np.load = lambda *a,**k: np_load_old(*a, allow_pickle=True, **k)
MapNp=np.load("Map_01.npz")
# call load_data with allow_pickle implicitly set to true
# # restore np.load for future normal usage
np.load = np_load_old

WaysList=[]  




def main():
    #Class Ways work with list not np arrays
    for way in MapNp["Paths"]:
        WaysList.append(way)
   
    Visualization=Visual(Map.Ways(MapNp["ObstacleList"]),WaysList,(150,100))


    cv2.waitKey(0)
    cv2.destroyAllWindows()

class Visual():
    def __init__(self,ways,pathlist,mapsize):
       
        map1=Map.Map(mapsize)
        #self.Image=cv2.imread("map_01.png")
        #eğer resimi kullanmak isterseniz aşşağıdaki 2 satırı yorum satırı yapıp üsttekini açın
        self.Image=np.zeros((map1.screensize[1],map1.screensize[0],3),np.uint8)
        self.Image[:]=[255,255,255]
        self.Ways=ways
        self.Ways.Path_List=pathlist
        self.path=[]
        self.waypoints=[]
        print("Başlangıç noktasının koordinatlarını giriniz:")
        Start_Point=Map.Points((int(input()),int(input())),self.Ways)
        self.waypoints.append(Start_Point)
        print("Uğramak istediğniz noktaların koordinatlarını giriniz:")
        print("Koordinat girmek istemiyorsanız 1 e basınız")
        while True:
            xaxis=int(input())
            if xaxis==1:break
            yaxis=input()
            points=Map.Points((int(xaxis),int(yaxis)),self.Ways)
            self.waypoints.append(points)
            print("devam etmek için koordinatları giriniz çıkmak için 1 e basınız")
        print("Bitiş noktasının koordinatlarını giriniz:")
        End_Point=Map.Points((int(input()),int(input())),self.Ways)
        self.waypoints.append(End_Point)
        self.drawlines(self.Image,self.Ways.Path_List,0,0,255,1)
        self.path,sum_cost=self.DijkstrasAlgorithm(self.Image,Start_Point,End_Point,self.waypoints)
        #self.findRoughness(map1)
        print("path",self.path,"path cost",sum_cost)
        #np.savez("deneme",path_points=self.path)
        print("--- %s seconds ---" % (time.time() - start_time))
               
    
  
               
    def drawlines(self,image,GivenLines=[],Blue=0,Green=0,Red=0,Thickness=2):
        for line in GivenLines:
            start=(int(line[0][0]),int(line[0][1]))
            end=(int(line[1][0]),int(line[1][1]))
            global start_time
            start_time = time.time()
            cv2.line(image,start,end,(Blue,Green,Red),Thickness)
            cv2.circle(image,start,1,(0,255,255),-1)
            cv2.circle(image,end,1,(0,255,255),-1)
    def drawcircle(self,image,GivenPoints,Radius,Blue,Green,Red,Thichkness):
        cv2.circle(image,GivenPoints,Radius,(Blue,Green,Red),Thichkness)
    
    def DijkstrasAlgorithm(self,image,Start_Point,End_Point,Waypoints=list):
        #waypointsi eklemeyi ayarla
        path=0
        if len(Waypoints)==1:
            path,sum_cost=self.CheckObstacles(Start_Point,End_Point,self.Ways)
            if path==[]:
                graph=PlanningAlgorithms.Graph_d()
                for point in self.Ways.Path_List:
                    graph.AddEdges(point[0],point[1],point[2])
                path,sum_cost=graph.CalculatePath(graph,Start_Point.Coord,End_Point.Coord)
            if not path=="Route Not Possible":
                self.drawlines(image,path,255,0,0,4)
            else:
                self.drawcircle(image,Start_Point.Coord,4,0,255,255,-1)
                self.drawcircle(image,End_Point.Coord,4,0,255,255,-1)  
            cv2.imshow("image",image)
            return path,sum_cost
        else:
           #calculate path from start point to waypoints
            value=0
            shortest_path=[] 
            costlist=[]
            total_cost=0
            graph1=PlanningAlgorithms.Graph_d()
            for point in self.Ways.Path_List:
                graph1.AddEdges(point[0],point[1],point[2])
            while value<len(Waypoints)-1:
                path,sum_cost=self.CheckObstacles(Waypoints[value],Waypoints[value+1],self.Ways)
                if path==[]:
                    path,sum_cost=graph1.CalculatePath(graph1,Waypoints[value].Coord,Waypoints[value+1].Coord)
                for points in path:
                    shortest_path.append(points)
                total_cost=total_cost+sum_cost
                value+=1
            if not path=="Route Not Possible":
                self.drawlines(image,shortest_path,255,0,0,4)
            else:
                self.drawcircle(image,Start_Point.Coord,4,0,255,255,-1)
                self.drawcircle(image,End_Point.Coord,4,0,255,255,-1)  
            cv2.imshow("image",image)
    
            
            return shortest_path,total_cost
 




    def CheckObstacles(self,startpoint,endpoint,ways):
        # if there is no obstacle between start and end points instead of using path algorithm the path are drawn as start and end point
        path=[]
        can_draw=True
        pairs=[startpoint.Coord,endpoint.Coord]
        if not startpoint.Roughness==endpoint.Roughness:
            return [],0  
        for obstacle in ways.ObstaclesList:
            for edge in obstacle.Edges:
                if MathAndOperations.MathOperations.doesCollide(self,pairs,edge):
                    can_draw=False
                    break
        if can_draw==True:
            path=[(pairs[0],pairs[1],MathAndOperations.MathOperations.LenghtOfPaths(self,pairs,roughness=startpoint.Roughness))]
            return path,path[0][2]
        return [],0

    def findRoughness(self,Map):
        #path cost ile pointlerin listesini aldım onları buraya import et sonra roughnessı yol uzunluğu/bulduğmuz uzunluk yaparak roughness bul
        index=0
        while index<len(self.path):
            lenght = round(np.sqrt(((self.path[index][1][1] - self.path[index][0][1]) ** 2) + ((self.path[index][1][0] - self.path[index][0][0]) ** 2)),2)
            if round(lenght,2)!=round(self.path[index][2],2):
                roughness=round(self.path[index][2]/lenght,2)
                
            else:
                roughness=0
            realworldlenght=round(lenght*Map.pixelsize,2)
            self.path[index][2]=lenght
            
            self.path[index].append(realworldlenght)
            self.path[index].append(roughness)
            index+=1
        









if __name__ == "__main__":
    main()
  




