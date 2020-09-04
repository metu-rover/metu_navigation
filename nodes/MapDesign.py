import numpy as np
import itertools

class MathOperations():
    
    def lenghtOfLines(self,Coords=[], roughness="Infinity"):
        # It is pythagorous' theorem
        if not roughness == "Infinity":return np.sqrt((((Coords[1][1] - Coords[0][1]) ** 2) + ((Coords[1][0] - Coords[0][0]) ** 2)))*roughness
        #If there is roughness, the line length is calculated by multiplying the length roughness.
        else:return np.sqrt(((Coords[1][1] - Coords[0][1]) ** 2) + ((Coords[1][0] - Coords[0][0]) ** 2)) 
    def doesCollide(self,Pairs=[()], Edges=[()]):
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
    def functionsOfEdges(self,Edges=[]):
        #f(x)=mx+a                     
        try:m=(Edges[1][1]-Edges[0][1])/(Edges[1][0]-Edges[0][0]) 
        except ZeroDivisionError:return 0,Edges[0][1]      #it is prevent the error of int/0 or 0/0 error and return constant value of function
        a=Edges[0][1]-(m*Edges[0][0])  #it gives constant value of function 
        return m,a   


ObstacleList=[] 
math=MathOperations() 
class Map():
    def __init__(self,mapsize=tuple,screensize=tuple):                     
        screenx,screeny,multiply=self.findMapSize(mapsize,screensize) 
        # it is for define a map that fit the screen but include as much pixel as could be and correct path length in world    
        self.imagesize=(screenx,screeny)   
        self.pixelsize=round(1/multiply,3) #every pixel in the image equals to this length in the world

    def findMapSize(self,mapsize,screensize):
        screenx,screeny=mapsize[0],mapsize[1]  
        multiply=2              #multiply of zoom
        if mapsize[0]<screensize[0] and mapsize[1]<screensize[1]:#if map is smaller than screensize the map is zoomed 
            while True:
                screenx=mapsize[0]*multiply                    
                screeny=mapsize[1]*multiply
                if screenx>screensize[0] or screeny>screensize[1]: #if zoomed mapsize is bigger than max screensize loop is broken
                    multiply-=1 #maximum multiply
                    break
                multiply+=1
            return mapsize[0]*multiply,mapsize[1]*multiply,multiply
        else:                                                      #if map is bigger than screensize the map is zoomed out 
            while True:
                screenx=mapsize[0]/multiply
                screeny=mapsize[1]/multiply
                if screenx<1920 or screeny<1080:                   #if zoomed mapsize is smaller than max screensize loop is broken
                    multiply-=1
                    break
                multiply+=1
            return int(mapsize[0]/multiply),int(mapsize[1]/multiply),1/multiply #to remove float numbers as result of dividing use int function

        



class Obstacles():

    def __init__(self,Coords,Roughness="Infinity"):
        global ObstacleList
        self.Coords=Coords   #coordinates must be defined sequentially
        self.InsideCoords=[]
        self.Edges=[]
        self.Roughness=Roughness
        self.MeanPoint=np.mean(self.Coords,axis=0) #to define outside and inside coords

        ObstacleList.append(self) #it is necessary for ways class

        self.CoordsToEdges() #define edges by using coords of obstacle
        if not self.Roughness == "Infinity": #to define ways in the obstacles that can passable
            self.InsideCoords = self.ShiftCoords(number=-1)
            self.InsideCoords.append((self.MeanPoint[0], self.MeanPoint[1]))
        self.Coords = self.ShiftCoords()

    def CoordsToEdges(self):
        index = 0
        while index!=len(self.Coords)-1:
            self.Edges.append((self.Coords[index],self.Coords[index+1]))   #each coordinate is combined with the next  
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
        #If the x-axis value of the coordinate is greater than the value of the mean point, this coordinate is to the right of the midpoint, 
        # so it must be shifted to the right for outside coordinates and must be shifred to the left for inside coordinates.
        




class Ways():

    def __init__(self,ObstaclesList=[]):
        self.ObstaclesList=ObstaclesList
        self.Pair_List=[] #to take combinations of the all obstacle's all points 
        self.Way_List=[]
    
    def Create_Ways(self,GivenPairs=[],Roughness=False):
        global math
        for pairs in GivenPairs: #given pairs include all combinations of all points
            can_draw=True
            for obstacle in self.ObstaclesList:
                for edge in obstacle.Edges: #if the pairs collide with any edge these pairs are deleted for path planning
                    if math.doesCollide(pairs,edge): #else add to possible accesable points
                        can_draw=False
                        break
            if can_draw==True:
                if not Roughness==False:
                    self.Way_List.append((pairs[0],pairs[1],math.lenghtOfLines(pairs,roughness=Roughness))) 
                    #If the obstacle has a roughness, the ways' lenght are added by multiplying their roughness 
                    #Becuse of the road roughness the vehicles must pass the area slower
                    # By using these the algorithm will choose most efficient way 
                else:
                    self.Way_List.append((pairs[0],pairs[1],math.lenghtOfLines(pairs)))
                    #the ways are added to list with their lenght  
                
    def Create_Rough_Pairlist(self):
        #These loops exist for crate a rough pair list which include only Rough coords for doesCollide function
        global math
        for obstacle in self.ObstaclesList:
            Rough_Pairlist=[]
            if not obstacle.Roughness=="Infinity":
                for pair in obstacle.InsideCoords: 
                    #In here the ways are drawn if an obstacle inside of another obstacle
                    for objects in self.ObstaclesList:
                        if not objects.Roughness=="Infinity":
                            for coords in objects.Coords:
                                Rough_Pairlist.append((pair,coords))
                                #until this part we included all combinatons of one obstacle's coord and other obstacle's insidecoord to list
                for inside in itertools.combinations(obstacle.InsideCoords,2):
                    Rough_Pairlist.append(inside)
                #in that part we included all combinations of one obstacle's coords between themselves to list for increase the ways that inside of obstacle
                for value,coord in enumerate(obstacle.Coords):
                    self.Way_List.append((coord,obstacle.InsideCoords[value],math.lenghtOfLines([coord,obstacle.InsideCoords[value]],roughness=obstacle.Roughness)))
                    #In order to the algorithm access to an obstacle which is passable the outside coords and inside coords of obstacle must be connected
                    #in that part we connected  inside coords and coords
                self.Create_Ways(Rough_Pairlist,obstacle.Roughness)
                #After create the pairs who have roughness the list is sent to create ways funciton
    
    def Crate_PairList(self):
        #The pair list contains all possible lines by the coordinate pairs generated. If these lines do not collide, they can be used as paths.
        PairList=[]
        for obstacle in self.ObstaclesList:
            for coords in obstacle.Coords:
                PairList.append(coords)       
        # It is for crate the list which contain all coords for take combinations of all of them 
        for pair in itertools.combinations(PairList, 2):
            self.Pair_List.append(pair)
            #itertools.combinations function returns all combinations of coordinate pairs of the paired list.

        



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
        global math
    
        PairList=[]
        for obstacle in GivenObstacleList:
            if not obstacle.Roughness=="Infinity":#the point can only be inside of the obstacles which have roughness.Otherwise the point will be inaccessable.
                counter=0                         #that is the reason why the function only checks obstacles that have roguhness.
                for edge in obstacle.Edges:
    
                    #The function create lines which has available equation from obstacle's edges
                    #By using these lines the function check the point is in the obstacle or not 
                    slope,constant=math.functionsOfEdges(edge)
                    y=(slope*obstacle.MeanPoint[0])+constant #the value of meanpoint if it is on the edge
                    y2=(slope*self.Coord[0])+constant        #the value of the point if it is on the edge
                    #f(x)=mx+a
                    if not slope==0:
                        try:
                            #x=(f(x)-a)/m
                            x=(obstacle.MeanPoint[1]-constant)/slope
                            x2=(self.Coord[1]-constant)/slope
                        except ZeroDivisionError:
                            x,x2=y,y2
                    #The meanpoint's coords are puted to equations of lines,if the value is smaller than meanpoint's originl value,the edge is upper than meanpoint so that 
                    #Coords must be below than the edges if it is inside of the obstacle 
                    if y<=obstacle.MeanPoint[1] and y2<=self.Coord[1]: 
                        counter+=1
                    elif y>=obstacle.MeanPoint[1] and y2>=self.Coord[1]:
                        counter+=1
                    elif x<=obstacle.MeanPoint[0] and x2<=self.Coord[0]:
                        counter+=1
                    elif x>=obstacle.MeanPoint[0] and x2>=self.Coord[0]:
                        counter+=1        
                    # If the point is correct side of edge counter value is increased. 
                if counter==len(obstacle.Edges):
                    #if counter's value is equal to number of edges,the point is in obstacle 
                    self.Roughness=obstacle.Roughness
                    return True                 
        else:return False
    
    def AddPointsToPairList(self,GivenObstacleList=[]):
        PairList=[]
        for obstacle in GivenObstacleList:
            for index,coord in enumerate(obstacle.Coords):
                PairList.append((self.Coord,coord)) #to create ways from the point to obstacles
                if not obstacle.Roughness=="Infinity":PairList.append((self.Coord,obstacle.InsideCoords[index]))
                #to create ways inside the obstacle if the point is inside
        return PairList





def main():
    global ObstacleList

    """""""""*************    IDENTIFIED   OBJECTS    *************"""""""""
    #do not define concave shapes,concave shapes must be divided into convex shapes and defined in this way
    #If the user wants to define an obstacle inside another obstacle,s/he must first define an inside one.
    Infinity1=Obstacles(((593, 545), (469, 644), (434, 665), (357, 663), (384, 599), (596, 497)))
    Infinity2=Obstacles(((99, 657), (95, 613), (108, 562), (194, 545), (328, 585), (342, 635), (303, 662)))
    Infinity3=Obstacles(((-41, 571), (83, 546), (60, 651)))
    Infinity4=Obstacles(((557, 423), (509, 495), (355, 492), (324, 450)))
    Infinity5=Obstacles(((557, 423), (567, 348), (441, 286), (511, 428)))
    Infinity6=Obstacles(((441, 286), (463, 332), (367, 412), (304, 418), (335, 307)))
    Infinity7=Obstacles(((517, 43), (431, 151), (328, 172), (330, 58), (486, -24)))
    Infinity8=Obstacles(((616, 152), (539, 50), (538, -118), (616, -202)))
    Infinity9=Obstacles(((527, 237), (519, 221), (485, 241), (498, 255)))
    Infinity10=Obstacles(((287, -12), (132, 84), (227, 95), (302, 30)))
    Infinity11=Obstacles(((132, 84), (191, 48), (152, -121), (106, 13)))
    Infinity12=Obstacles(((231, 268), (186, 177), (85, 331), (107, 418)))
    Infinity13=Obstacles(((159, 220), (186, 177), (110, 112), (12, 111)))
    Infinity14=Obstacles(((12, 111), (66, 150), (0, 190)))
    Infinity15=Obstacles((((12, 111), (-76, 199), (-86, 275), (-46, 349), (-18, 330))))
    Infinity16=Obstacles(((50, 310), (-2, 358), (65, 395)))
    Infinity17=Obstacles(((113, -209), (32, -57), (-53, -14), (-76, -73), (24, -247)))
    Infinity18=Obstacles(((77, -249), (71, -314), (133, -314)))
    Infinity19=Obstacles(((177, -164), (145, -168), (208, -314), (267, -314)))
    
    Rough1=Obstacles(((493, -314), (483, -150), (400, -81), (316, -100), (283, -201), (316, -314)),Roughness=4)
    Rough2=Obstacles(((596, 497), (472, 553), (557, 423)),Roughness=1.2)
    Rough3=Obstacles(((472, 553), (371, 597), (294, 518), (355, 492), (509, 495)),Roughness=1.2)
    Rough4=Obstacles(((294, 518), (355, 492), (304, 418), (250, 446)),Roughness=1.2)
    Rough5=Obstacles(((232, 522), (111, 561), (147, 515), (229, 475)),Roughness=1.5)
    Rough6=Obstacles(((147, 470), (83, 546), (26, 558), (-60, 508), (-7, 383)),Roughness=2)
    Rough7=Obstacles(((-7, 383), (-60, 508), (-193, 443), (-193, 283)),Roughness=2)
    Rough8=Obstacles(((-60, 508), (-81, 545), (-170, 557), (-175, 517), (-130, 475)),Roughness=6)
    Rough9=Obstacles(((-193, 155), (-129, -122), (-193, -314)),Roughness=2.5)
    Rough10=Obstacles(((527, 237), (519, 221), (616, 151), (616, 326)),Roughness=1.3)
    """""""""*************    IDENTIFIED   OBJECTS    *************"""""""""


    
    Way=Ways(ObstacleList)

    Way.Create_Rough_Pairlist()
    Way.Crate_PairList()
    Way.Create_Ways(Way.Pair_List)
    np.savez("leo_map",ObstacleList=ObstacleList,Paths=Way.Way_List)
    #with np.savez the map and obstacles define one time and until the user wants changed something in map program doesn't have to use MapDesign.
    # To sum up it increases program performance. 
   
 
    
   

   


if __name__ == "__main__":
    main()





