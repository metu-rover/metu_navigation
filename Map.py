import numpy as np
import MathAndOperations
import time

import warnings
warnings.filterwarnings("ignore", category=np.VisibleDeprecationWarning) 

start_time = time.time()
ObstacleList=[]
class Map():
    def __init__(self,mapsize=tuple):
        screenx,screeny,multiply=self.findMapSize(mapsize)
        self.screensize=(screenx,screeny)
        self.pixelsize=round(1/multiply,3)

    def findMapSize(self,mapsize):
        screenx,screeny=mapsize[0],mapsize[1]
        multiply=2
        if mapsize[0]<1920 and mapsize[1]<1080:
            while True:
                screenx=mapsize[0]*multiply
                screeny=mapsize[1]*multiply
                if screenx>1920 or screeny>1080:
                    multiply-=1
                    break
                multiply+=1
            return mapsize[0]*multiply,mapsize[1]*multiply,multiply
        else:
            while True:
                screenx=mapsize[0]/multiply
                screeny=mapsize[1]/multiply
                if screenx<1920 or screeny<1080:
                    multiply-=1
                    break
                multiply+=1
            return int(mapsize[0]/multiply),int(mapsize[1]/multiply),1/multiply

        



class Obstacles():

    def __init__(self,Coords,Roughness="Infinity"):
        global ObstacleList

        self.Coords=Coords
        self.InsideCoords=[]
        self.Edges=[]
        self.Roughness=Roughness
        self.MeanPoint=np.mean(self.Coords,axis=0)

        ObstacleList.append(self)

        self.CoordsToEdges()
        if not self.Roughness == "Infinity":
            self.InsideCoords = self.ShiftCoords(number=-1)
            self.InsideCoords.append((self.MeanPoint[0], self.MeanPoint[1]))
        self.Coords = self.ShiftCoords()

    def CoordsToEdges(self):
        index = 0
        while index!=len(self.Coords)-1:
            self.Edges.append((self.Coords[index],self.Coords[index+1]))         
            index+=1
        self.Edges.append((self.Coords[index],self.Coords[0]))
       

    def ShiftCoords(self,number=1):
        new_coords=[]

        #Coords must be shifted for do not collide with any own edges
        for coord in self.Coords:
            if coord[0]>self.MeanPoint[0]:
                new_coords_x=coord[0]+number
            else:
                new_coords_x=coord[0]-number
            if coord[1] >self.MeanPoint[1]:
                new_coords_y=coord[1]+number
            else:
                new_coords_y=coord[1]-number
           
            new_coords.append((new_coords_x,new_coords_y))
        return new_coords
        #if x,y of coord are bigger than the meanpoint coords must be shifted right,down              
        




class Ways():

    def __init__(self,ObstaclesList=[]):
        self.ObstaclesList=ObstaclesList
        self.Pair_List=[]
        self.Path_List=[]

        self.Create_Rough_Ways()
        self.Crate_PairList()
        self.Create_Ways(self.Pair_List)
    
    def Create_Ways(self,GivenPairs=[],Roughness=False):
        for pairs in GivenPairs:
            can_draw=True
            index=0
            value=0
            for obstacle in self.ObstaclesList:
                for edge in obstacle.Edges:
                    if MathAndOperations.MathOperations.doesCollide(self,pairs,edge):
                        can_draw=False
                        break
            if can_draw==True:
                if not Roughness==False:
                    self.Path_List.append((pairs[0],pairs[1],MathAndOperations.MathOperations.LenghtOfPaths(self,pairs,roughness=Roughness)))
                    
                else:
                    self.Path_List.append((pairs[0],pairs[1],MathAndOperations.MathOperations.LenghtOfPaths(self,pairs)))
                
    def Create_Rough_Ways(self):
        #These loops exist for crate a rough pair list which include only Rough coords for doesCollide function
        
        for obstacle in self.ObstaclesList:
            Rough_Pairlist=[]
            if not obstacle.Roughness=="Infinity":
                for pair in obstacle.InsideCoords:
                    for objects in self.ObstaclesList:
                        if not objects.Roughness=="Infinity":
                            for coords in objects.Coords:
                                Rough_Pairlist.append((pair,coords))
                                #until this part we included all combinatons of one obstacle's coord and other obstacle's insidecoord to list
                for inside in MathAndOperations.itertools.combinations(obstacle.InsideCoords,2):
                    Rough_Pairlist.append(inside)
                #in that part we included all combinations of one obstacle's coords between themselves to list
                for value,coord in enumerate(obstacle.Coords):
                    self.Path_List.append((coord,obstacle.InsideCoords[value],MathAndOperations.MathOperations.LenghtOfPaths(self,[coord,obstacle.InsideCoords[value]],roughness=obstacle.Roughness)))
                    
                    #in that part we connected  inside coords and coords
                self.Create_Ways(Rough_Pairlist,obstacle.Roughness)
        
    
    def Crate_PairList(self):
        #Pair list is created for control all possible lines if they can drawn or not
        PairList=[]
        for obstacle in self.ObstaclesList:
            for coords in obstacle.Coords:
                PairList.append(coords)       
        # It is for crate the list which contain all coords
        for pair in MathAndOperations.itertools.combinations(PairList, 2):
            self.Pair_List.append(pair)

        



class Points():
    
    def __init__(self,coords,Way):
        self.Coord=coords
        self.Roughness="Infinity"

        check=self.IsItInside(Way.ObstaclesList)
        PairList=self.AddPointsToPairList(Way.ObstaclesList)
        if check==True:
            Way.Create_Ways(PairList,self.Roughness)
        else:
            Way.Create_Ways(PairList)
    
    def IsItInside(self,GivenObstacleList=[]):
        #The function checks if the point is in the obstacle
    
        PairList=[]
        for obstacle in GivenObstacleList:
            if not obstacle.Roughness=="Infinity":
                counter=0
                for edge in obstacle.Edges:
                    #create lines which has available equation from obstacle's edges
                    slope,constant=MathAndOperations.MathOperations.FunctionsOfEdges(self,edge)
                    y=(slope*obstacle.MeanPoint[0])+constant
                    y2=(slope*self.Coord[0])+constant
                    #f(x)=mx+a
                    if not slope==0:
                        try:
                            #x=y-a/m
                            x=(obstacle.MeanPoint[1]-constant)/slope
                            x2=(self.Coord[1]-constant)/slope
                        except ZeroDivisionError:
                            x,x2=y,y2
                    """ put the meanpoint's coords to equations,if the value is bigger than meanpoint's originl value,the edge is upper than meanpoint so that 
                    self.Coords must below than the edges if is inside of the obstacle """
                    if y<=obstacle.MeanPoint[1] and y2<self.Coord[1]:  
                        counter+=1
                    elif y>obstacle.MeanPoint[1] and y2>self.Coord[1]:
                        counter+=1
                    elif x<obstacle.MeanPoint[0] and x2<self.Coord[0]:
                        counter+=1
                    elif x>obstacle.MeanPoint[0] and x2>self.Coord[0]:
                        counter+=1        
                if counter==len(obstacle.Edges):
                    #if counter's value is equal to number of edges,the point is on obstacle 
                        
                    self.Roughness=obstacle.Roughness
                    return True                 
        else:
            return False
    
    def AddPointsToPairList(self,GivenObstacleList=[]):
        PairList=[]
        
        for obstacle in GivenObstacleList:
            for coord in obstacle.Coords:
                PairList.append((self.Coord,coord))
            for coord in obstacle.InsideCoords:  
                PairList.append((self.Coord,coord))
        return PairList





def main():
    global ObstacleList

    """""""""*************    IDENTIFIED   OBJECTS    *************"""""""""
    #do not define concave shapes,concave shapes must be divided into convex shapes and defined in this way
    Infinity1=Obstacles([(270, 202), (287, 258), (356, 316), (307, 399), (229, 416),(172, 349)])

 
    Infinity2=Obstacles([(138, 626), (256, 748), (346, 752), (480, 906), (307, 1004), (73, 947), (49, 796)])


    Infinity3=Obstacles([(402, 708), (358, 570), (463, 404), (640, 209), (795, 191), (833, 241), (857, 385), (783, 461)])


    Infinity4=Obstacles( [(664, 652), (747, 712), (864, 659), (1084, 703), (1118, 823), (1080, 975), (724, 962), (488, 813)])


    Infinity5=Obstacles( [(984, 118), (1093, 95), (1115, 238), (1023, 287),(947, 252)])


    Infinity6=Obstacles([(1025, 391), (1191, 337), (1296, 361), (1353, 493),(1153, 663), (1025, 579)])

    




    Rough1=Obstacles([(897,423),(926,495),(986,461),(975,418)],Roughness=3)


    Rough2=Obstacles([(147,455),(219,457),(203,551),(166,545)],Roughness=4)


    Rough3=Obstacles([(706,525),(797,630),(1017,617),(997,397),(908,365)],Roughness=1.8)

    

    Rough4=Obstacles([(85,503),(124,424),(278,451),(250,596),(132,578)],Roughness=2)


  

   
    
    

    """""""""*************    IDENTIFIED   OBJECTS    *************"""""""""



    Way=Ways(ObstacleList)
    np.savez("Map_01",ObstacleList=ObstacleList,Paths=Way.Path_List)
   
 
    
   

    print("--- %s seconds ---" % (time.time() - start_time))


if __name__ == "__main__":
    main()





