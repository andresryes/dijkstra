'''
References:
https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm
https://dev.to/mxl/dijkstras-algorithm-in-python-algorithms-for-beginners-dkc
'''
#imports
import ast
from collections import deque, namedtuple

#defining the infinte value for the alg.
inf = float('inf')
#template for an edge
Edge = namedtuple('Edge', 'start, end, cost')

#list to append the nodes
graph_list= []
#read the txt and append the nodes 
file_text=open('graph.txt','r')
for line in file_text.readlines():
    #remove the spaces 
    line1 = line.replace("\n", "")
    node = ast.literal_eval(line1)
    #appending the node the list
    graph_list.append(node)
#close the file
file_text.close()

    
#method to create the edges 
def make_edge(start, end, cost=1):
    return Edge(start, end, cost)

#class for the graph
class Graph:
    #to instance a class of the type  Graph
    #receives a list with the nodes and if it has direction
    #if it has direction, direction=True
    def __init__(self, edges, direction):
        self.edges = [make_edge(*edge) for edge in edges] 
        if direction == 1:
            self.direction = True
        else:
            self.direction = False

    #this piece of magic turns ([1,2], [3,4]) into [1, 2, 3, 4]
    #the set above makes it's elements unique.
    @property
    def vertices(self):
        return set(
            sum(
                ([edge.start, edge.end] for edge in self.edges), []
            )
        )

    #making the ordered pairs
    def get_node_pairs(self, node_1, node_2):
        if self.direction:
            node_pairs = [[node_1, node_2], [node_2, node_1]]
        else:
            node_pairs = [[node_1, node_2]]
        return node_pairs

    #remove and edge if it's necessary  
    def remove_edge(self, node_1, node_2):
        node_pairs = self.get_node_pairs(node_1, node_2)
        edges = self.edges[:]
        for edge in edges:
            if [edge.start, edge.end] in node_pairs:
                self.edges.remove(edge)

    #adding functionality 
    def add_edge(self, node_1, node_2, cost=1):
        node_pairs = self.get_node_pairs(node_1, node_2)
        for edge in self.edges:
            if [edge.start, edge.end] in node_pairs:
                return ValueError('Edge {} {} already exists'.format(node_1, node_2))

        self.edges.append(Edge(start=node_1, end=node_2, cost=cost))
        if self.direction:
            self.edges.append(Edge(start=node_2, end=node_1, cost=cost))

    #find the neighbours for every node 
    @property
    def neighbours(self):
        neighbours = {vertex: set() for vertex in self.vertices}
        for edge in self.edges:
            neighbours[edge.start].add((edge.end, edge.cost))

        return neighbours

    def dijkstra(self, source, dest):
        #1. Mark all nodes unvisited and store them
        #2. Set the distance to zero for our initial node 
        #and to infinity for other nodes

        distances = {vertex: inf for vertex in self.vertices}

        #previous vertices 
        previous_vertices = {
            vertex: None for vertex in self.vertices
        }

        #set initial distance
        distances[source] = 0
        vertices = self.vertices.copy()

        #while there are at least one vertex in the list
        while vertices:
            #3. Select the unvisited node with the smallest distance, 
            #it's current node now
            #set the current node 
            current_vertex = min(
                vertices, key=lambda vertex: distances[vertex])
            #remove it from the temp list, since it has been used already
            vertices.remove(current_vertex)

            # 6. Stop, if the smallest distance 
            # among the unvisited nodes is infinity
            #if the distances of current is inf, end the while
            if distances[current_vertex] == inf:
                break

            # 4. Find unvisited neighbors for the current node 
            # and calculate their distances through the current node
            #verify to set the new current node
            for neighbour, cost in self.neighbours[current_vertex]:
                alternative_route = distances[current_vertex] + cost
                if alternative_route < distances[neighbour]:
                    distances[neighbour] = alternative_route
                    previous_vertices[neighbour] = current_vertex

            # 5. Mark the current node as visited 
            # and remove it from the unvisited set.
            #vertices.remove(current_vertex)

        #vertices that are in the list for the path
        #are the nodes for the best path
        #return the total cost for that path
        path, current_vertex = deque(), dest
        while previous_vertices[current_vertex] is not None:
            cost = distances[current_vertex]
            path.appendleft(current_vertex)
            current_vertex = previous_vertices[current_vertex]
        if path:
            path.appendleft(current_vertex)
        return path

#method to calculate the cost given an specific path in
#a graph
def calculate_cost(path, graph):
    cost = 0
    for node in range(len(path)-1):
        for vertex in graph:
            if vertex[0] == path[node] and vertex[1]==path[node+1]:
                cost+=vertex[2]
    return cost

#set the graph with the list from the txt
#if the graph has direction, the second param must be True
direction = input("Does the graph have direction? 1 = True, 0 = False\n")
graph = Graph(graph_list, int(direction))

#calculate the path
path = graph.dijkstra("a", "e")

#calculate the cost for that specific path in the graph
cost = calculate_cost(path, graph_list)

print("The path is:" + str(list(path)))
print("With a total cost of: "+str(cost))