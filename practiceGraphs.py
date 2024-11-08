## same file as 'graphs.py' but with print statements to help understand intervals of code; 'graphs.py' is the clean copy
# *** Routes algorithm not fully correct yet; tends to yield errors on routes with > 3 steps. ***
# ** Needs recursive functions/quality check to weed out bugs. Will implemented at a later date. ** 

import heapq

class Graph:
    def __init__(self):
        self.vertices = {}

    def add_vertex(self, vertex):
        if vertex not in self.vertices:
            self.vertices[vertex] = {}

    def add_edge(self, vertex1, vertex2, weight):
        if vertex1 in self.vertices and vertex2 in self.vertices:
            self.vertices[vertex1][vertex2] = weight
            self.vertices[vertex2][vertex1] = weight

    def get_neighbors(self, vertex):
        if vertex in self.vertices:
            return self.vertices[vertex]
        else:
            return {}

    def dijkstra(self, start):
        distances = {vertex: float('inf') for vertex in self.vertices}
        paths = {}
        distances[start] = 0
        pq = [(0,start)]

        while pq:
            print('while begin')
            print('**pq start:', pq)
            current_distance, current_vertex = heapq.heappop(pq)
            if current_distance > distances[current_vertex]:
                print('Current Distance:', current_distance)
                print('distances[current_vertex]:', distances[current_vertex])
                continue
            print('Vertex:', current_vertex)
            print(self.vertices[current_vertex])
            for neighbor, weight in self.vertices[current_vertex].items():  
                route = []    
                print('\n-> Neighbor:', neighbor)
                print('\nWeight:', weight)
                print('Current Distance:', current_distance)
                distance = current_distance + weight
                print(f'distance: {distance} | distances[neighbor]: {distances[neighbor]}')
                if distance < distances[neighbor]:
                    
                    print('Vertex:', current_vertex)
                    print(self.vertices[current_vertex])
                    if start != current_vertex:
                        step = {current_vertex: current_distance}

                        print('step1:', step)
                        if start not in step:
                            route.append(step)
                        step = {neighbor: weight}
                        print('step2:', step)
                        if start not in step:
                            route.append(step)
                    else:
                        step = {neighbor: weight}
                        print('step2:', step)
                        if start not in step:
                            route.append(step)
                    paths[neighbor] = route
                    print('paths:', paths)
                    distances[neighbor] = distance
                    print('distances:', distances)
                    heapq.heappush(pq, (distance, neighbor))
        print(paths)
        paths = dict(sorted(paths.items()))
        print(f'Paths from {start}:', paths)
        return distances, paths

graph = Graph()

graph.add_vertex('A')
graph.add_vertex('B')
graph.add_vertex('C')
graph.add_vertex('D')
graph.add_vertex('E')
graph.add_vertex('F')
graph.add_vertex('G')

graph.add_edge('A', 'B', 7)
graph.add_edge('B', 'C', 2)
graph.add_edge('A', 'C', 8)
graph.add_edge('F', 'D', 12)
graph.add_edge('B', 'E', 10)
graph.add_edge('D', 'E', 4)
graph.add_edge('F', 'B', 2)
graph.add_edge('G', 'C', 5)
# Vertices
# {'A': {'B': 7, 'C': 8}, 
# 'B': {'A': 7, 'C': 2, 'E': 10, 'F': 2}, 
# 'C': {'B': 2, 'A': 8, 'G': 5}, 
# 'D': {'F': 12, 'E': 4}, 
# 'E': {'B': 10, 'D': 4}, 
# 'F': {'D': 12, 'B': 2}, 
# 'G': {'C': 5}}
start = 'G'
end = 'A'
print('Vertices:', graph.vertices)
distances, paths = graph.dijkstra(start)
print('\nMinimum distances between points:', distances)
def pretty_paths(paths):
    print('\nPaths:')
    for path in paths:
        if len(paths[path]) > 1:
            i = 0
            for step in paths[path]:
                if path not in step:
                    step = paths[list(step.keys())[0]]
                    paths[path][i] = step
                    del paths[path][i]
                    for item in step:
                        j=0
                        paths[path].insert(j, item)
                        j+=1
                    i+=1
        print(f'{path} - {paths[path]}')
    print(paths)
pretty_paths(paths)
def display_route(start, end, paths):
    if end not in paths:
        print('\nThe end point is not within the paths database. Please try again!')
    else:
        i=1
        print('\n-*-*-Route-*-*-')
        print(f'\nStart: {start}. End: {end}')
        total = 0
        edge0 = start
        for item in paths[end]:
            for point, distance in item.items():
                edge1 = point
                print(f'{i}. Take {edge0}{edge1} - {distance} units.')
                total += distance
                edge0 = point
                i+=1
        print(f'{i}. Arrive at {end}.')
        print(f'Total distance: {total} units.')
        print('Thanks for using Routes!\n')
display_route(start, end, paths)