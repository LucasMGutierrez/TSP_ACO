import random
from copy import copy

class Graph:

    def __init__(self, directed = False, completed = False, randweight = None, numvertices = None):
        self.adjList = dict()
        self.directed = directed
        if numvertices != None:
            for i in range(numvertices):
                self.addVertice(i)

        if completed and numvertices != None:
            for i in range(numvertices):
                for j in range(numvertices):
                    if i != j:
                        if randweight != None:
                            self.addEdge(i, j, random.randint(1, randweight))
                        else:
                            self.addEdge(i, j, 1)


    def addVertice(self, vertice):
        self.adjList[vertice] = list()

    def addEdge(self, v1, v2, weight = 1):
        if self.getEdge(v1, v2) == None:
            self.adjList[v1].append((v2, weight))

            if not self.directed:
                self.adjList[v2].append((v1, weight))

    def degree(self, vertice):
        return len(self.adjList[vertice])

    def getEdge(self, v1, v2):
        for v in self.adjList[v1]:
            if v[0] == v2:
                return v

        return None
    
    def getEdges(self, v):
        return self.adjList[v]

    def getWeight(self, v1, v2):
        v = self.getEdge(v1, v2)
        if v != None:
            return v[1]

        return None

    def getVertices(self):
        return list(self.adjList.keys())

    def numVertices(self):
        return len(self.adjList)

    def pathSize(self, vertices):
        size = 0

        for i in range(len(vertices)-1):
            size += self.getWeight(vertices[i], vertices[i+1])

        size += self.getWeight(vertices[0], vertices[len(vertices)-1])

        return size


    def minimalPath(self, first, vertice = None, path = None, minpath = None):
        if path == None:
            path = []
            vertice = first

        path.append(vertice)

        if len(path) == self.numVertices():
            #print(path, self.pathSize(path))
            if minpath == None or self.pathSize(path) < self.pathSize(minpath):
                minpath = copy(path)

            return minpath

        for (v,_) in self.adjList[vertice]:
            if v not in path:
                minpath = self.minimalPath(first, v, path, minpath)
                path.pop(len(path) - 1)

        return minpath

# Exemplo

#graph = Graph()
#
#graph.addVertice(0)
#graph.addVertice(1)
#graph.addVertice(2)
#graph.addVertice(3)
#
#graph.addEdge(0, 1, 2)
#graph.addEdge(1, 3, 3)
#graph.addEdge(2, 3, 4)
#graph.addEdge(2, 1, 5)
#
#print(graph.adjList)
#print(graph.getWeight(1,0))

graph = Graph(completed = True, numvertices = 40, randweight = 10)
print(graph.adjList)
print(graph.minimalPath(0))
