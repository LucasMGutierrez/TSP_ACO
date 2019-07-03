import random
from copy import copy

class Graph:

    def __init__(self, directed = False, completed = False, randweight = None, numvertices = None, filename = None):
        self.adjList = dict()
        self.directed = directed
        if filename != None:
            self.load(filename)
            return

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

    def restart(self):
        self.adjList = dict()

    def save(self, filename):
        f = open(filename, mode = 'w')

        f.write(str(int(self.directed)) + "\n")

        for k in self.adjList:
            f.write(str(k))
            for (v, w) in self.adjList[k]:
                f.write(" " + str(v))
                f.write(" " + str(w))

            f.write("\n")
        f.close()

    def load(self, filename):
        self.restart()
        f = open(filename, mode = 'r')
        dat = f.read()
        f.close()
        dat = dat.split('\n')

        if dat[len(dat) - 1] == "":
            dat.pop()

        self.directed = bool(int(dat[0]))
        dat.pop(0)
        print(dat)

        for line in dat:
            l = line.split(" ")
            v = l[0]
            if v.isnumeric():
                v = int(v)
            self.addVertice(v)

        for line in dat:
            line = line.split(" ")
            i = 1
            v = line[0]
            if v.isnumeric():
                v = int(v)
            while i < len(line):
                v2 = line[i]
                if v2.isnumeric():
                    v2 = int(v2)
                i += 1
                w = int(line[i])
                self.addEdge(v, v2, w)
                i += 1


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

#graph = Graph(completed = True, numvertices = 4, randweight = 10)
graph = Graph(filename = "salvo")
print(graph.adjList)
