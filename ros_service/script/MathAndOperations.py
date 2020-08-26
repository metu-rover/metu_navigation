import numpy as np
import itertools
import math



class MathOperations():
    
    def lenghtOfLines(self,Coords=[], roughness="Infinity"):
        # It is pythagorous' theorem
        if not roughness == "Infinity":return np.sqrt((((Coords[1][1] - Coords[0][1]) ** 2) + ((Coords[1][0] - Coords[0][0]) ** 2)))*roughness
        #If there is roughness, the line length is calculated by multiplying the length roughness.
        else:return np.sqrt(((Coords[1][1] - Coords[0][1]) ** 2) + ((Coords[1][0] - Coords[0][0]) ** 2)) 
    def doesCollide(self,Pairs=[()], Edges=[()]):
        #Original form of the function is under the all functions
        #Does collide function is used for check two lines if they collide or not
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
    def functionsOfEdges(self,Edges=[]):
        #f(x)=mx+a                     
        try:m=(Edges[1][1]-Edges[0][1])/(Edges[1][0]-Edges[0][0]) 
        except ZeroDivisionError:return 0,Edges[0][1]      #it is prevent the error of int/0 or 0/0 error and return constant value of function
        a=Edges[0][1]-(m*Edges[0][0])  #it gives constant value of function 
        return m,a                          
  
        






"""""""""
It is some math equations that check if two straight line collide or not.Actually i do not know how it is work.I copied from net.

def doesCollide(x1, y1, x2, y2, x3, y3, x4, y4):
    if ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1)) == 0:
        return False
    uA = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1))
    uB = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1))

    if (uA >= 0 and uA <= 1 and uB >= 0 and uB <= 1):
        return True
    return False
"""""""""






