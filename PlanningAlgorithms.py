
from collections import defaultdict
import time



class Dijkstra():
    def CalculatePath(self,graph, initial, end):
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
                roughness = graph.PointRoughnesseses[(current_node, next_node)] + roughness_to_current_node
                if next_node not in shortest_paths:
                    shortest_paths[next_node] = (current_node, roughness)
                else:
                    current_shortest_roughness = shortest_paths[next_node][1]
                    if current_shortest_roughness > roughness:
                        shortest_paths[next_node] = (current_node, roughness)
            next_destinations = {node: shortest_paths[node] for node in shortest_paths if node not in visited}
            if not next_destinations:
                return "Route Not Possible","There is no Path Cost"
            # next node is the destination with the lowest roughness
            current_node = min(next_destinations, key=lambda k: next_destinations[k][1])
       

        # Work back through destinations in shortest path
        path_points = []
        sum_cost = 0
        path_cost=[]
        while current_node is not None:
            if sum_cost == 0:
                sum_cost = shortest_paths[current_node][1]

            path_points.append(current_node)
            path_cost.append(shortest_paths[current_node][1])
            next_node = shortest_paths[current_node][0]
            current_node = next_node
            
        # Reverse path
        path_points = path_points[::-1]
        path=[]
        path_cost=path_cost[::-1]
        for index, points in enumerate(path_points):
            if index == len(path_points) - 1: break
            point_cost=path_cost[index+1]-path_cost[index]
            path.append([points,path_points[index+1],point_cost])

        return path,sum_cost


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






    
    
    
    

