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
            route = []
            current_distance, current_vertex = heapq.heappop(pq)
            if current_distance > distances[current_vertex]:
                continue
            for neighbor, weight in self.vertices[current_vertex].items():       
                distance = current_distance + weight
                if distance < distances[neighbor]:
                    step = {current_vertex: current_distance}
                    if start not in step:
                        route.append(step)
                    step = {neighbor: weight}
                    if start not in step:
                        route.append(step)
                    paths[neighbor] = route
                    distances[neighbor] = distance
                    heapq.heappush(pq, (distance, neighbor))
            
        for item in paths:
            if start in self.vertices[item]:
                paths[item] = [{item: self.vertices[item][start]}]
        return distances, paths

graph = Graph()

graph.add_vertex('A')
graph.add_vertex('B')
graph.add_vertex('C')
graph.add_vertex('D')
graph.add_vertex('E')

graph.add_edge('A', 'B', 7)
graph.add_edge('B', 'C', 2)
graph.add_edge('A', 'C', 8)
graph.add_edge('A', 'D', 12)
graph.add_edge('B', 'E', 10)
graph.add_edge('D', 'E', 4)

start = 'E'
end = 'A'
distances, paths = graph.dijkstra(start)
print('\nMinimum distances between points:', distances)
print(f'Paths between start point {start}: {paths}')
def display_route(start, end, paths):
    if end not in paths:
        print('\nThe end point is not within the paths database. Please try again!')
    else:
        i=1
        print(f'\nStart: {start}.')
        edge0 = start
        for item in paths[end]:
            for point, distance in item.items():
                edge1 = point
                print(f'{i}. Take {edge0}{edge1} - {distance} units.')
                edge0 = point
                i+=1
        print(f'{i}. Arrive at {end}.')
        print('Thanks for using Routes!\n')
display_route(start, end, paths)