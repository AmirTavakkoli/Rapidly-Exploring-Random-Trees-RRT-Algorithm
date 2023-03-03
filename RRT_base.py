import os
import math
import numpy as np
import matplotlib.pyplot as plt

import random
import pygame


# clear = lambda: os.system('cls')  # On Windows System
# clear()

# Rapidly Exploring Random Tress (RRT)


# mapp class: containing methods to draw the map
class RRTMap:
    def __init__(self, start, goal, MapDimensions, obsdim, obsnum):
        self.start = start
        self.goal = goal
        self.MapDimensions = MapDimensions
        # Map dimensions contain two factors known as height and width
        self.Maph, self.Mapw =  self.MapDimensions
        self.obsdim = obsdim
        self.obsnum = obsnum

        # window settings
        self.MapWindowName = 'RRT Path Planning'
        pygame.display.set_caption(self.MapWindowName)
        self.map = pygame.display.set_mode((self.Mapw, self.Maph)) # height and width as tuple
        self.map.fill((255, 255, 255)) # fill each with white color using RGB codes
        self.nodeRad = 2
        self.nodeThickness = 0
        self.edgeThickness = 1

        # predefining obstacle list
        self.obstacles = []
        self.obsdim = obsdim
        self.obsNumber = obsnum

        # colors using RGB codes
        self.grey = (70, 70, 70)
        self.Blue = (0, 0, 255)
        self.Green = (0, 255, 0)
        self.Red = (255, 0, 0)
        self.white = (255, 255, 255)

    # drwaing the map, start and final points and their assigned color and sizes
    def drawMap(self, obstacles):
        pygame.draw.circle(self.map, self.Green, self.start, self.nodeRad + 5, 0)
        pygame.draw.circle(self.map, self.Green, self.goal, self.nodeRad + 20, 1)
        self.drawObs(obstacles)

    # drawing the path
    def drawPath(self, path):
        for node in path:
            pygame.draw.circle(self.map, self.Red, node, 3, 0)

    # dewaring obstacles as reactangles
    def drawObs(self, obstacles):
        obstaclesList = obstacles.copy()
        while (len(obstaclesList) > 0):
            obstacle = obstaclesList.pop(0)
            pygame.draw.rect(self.map, self.grey, obstacle)



# graph class: making obstacles, adding and removing nodes, number of nodes, distance, path-
# etc. 
class RRTGraph:
    def __init__(self, start, goal, MapDimensions, obsdim, obsnum):
        (x, y) = start
        self.start = start
        self.goal = goal
        self.goalFlag = False
        self.maph, self.mapw = MapDimensions
        self.x = []
        self.y = []
        self.parent = []

        # initializing the tree
        self.x.append(x)
        self.y.append(y)
        self.parent.append(0)

        # the obstacle
        self.obstacles = []
        self.obsDim = obsdim
        self.obsNum = obsnum

        # path
        self.goalstate = None
        self.path = []

    # making random recangles in scene
    def MakeRandomRect(self):
        uppercornerx = int(random.uniform(0, self.mapw - self.obsDim))
        uppercornery = int(random.uniform(0, self.maph - self.obsDim))

        return (uppercornerx, uppercornery)

    # making obstacles
    def makeobs(self):
        obs = []
        for i in range(0, self.obsNum):
            rectang = None
            startgoalcol = True
            while startgoalcol:
                upper = self.MakeRandomRect()
                rectang = pygame.Rect(upper, (self.obsDim, self.obsDim))
                # Collision
                if rectang.collidepoint(self.start) or rectang.collidepoint(self.goal):
                    startgoalcol = True
                else:
                    startgoalcol = False
            obs.append(rectang)
        self.obstacles = obs.copy()
        return obs

    # adding node
    def add_node(self, n, x, y):
        self.x.insert(n, x)
        self.y.append(y)

    # removing node
    def remove_node(self, n):
        self.x.pop(n)
        self.y.pop(n)

    # adding child to a parent (edge)
    def add_edge(self, parent, child):
        self.parent.insert(child, parent) # assinging child to a parent

    # removing edge
    def remove_edge(self, n):
        self.parent.pop(n) # cut the relationship between the parent and the child node

    # number of nodes
    def number_of_nodes(self):
        return len(self.x)

    # calculating the distance using euclidean method
    def distance(self, n1, n2):
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])
        px = (float(x1) - float(x2)) ** 2
        py = (float(y1) - float(y2)) ** 2
        return (px + py) ** (0.5)


   # generating random samples from the map
    def sample_envir(self):
        x = int(random.uniform(0, self.mapw))
        y = int(random.uniform(0, self.maph))
        return x, y

    # finding nearest node that does not cross any objects
    def nearest(self, n):
        dmin = self.distance(0, n)
        nnear = 0
        for i in range(0, n):
            if self.distance(i, n) < dmin:
                # updating distance to the new value of minimum distance
                dmin = self.distance(i, n)
                nnear = i
        return nnear


    # testing if the new passed node is located in a free space
    def isFree(self):
        n = self.number_of_nodes() - 1
        (x, y) = (self.x[n], self.y[n])
        obs = self.obstacles.copy()
        while len(obs) > 0:
            rectang = obs.pop(0)
            # Collision
            if rectang.collidepoint(x, y):
                self.remove_node(n)
                return False
        return True

    # find out if a connection between any two nodes crosses any obstacles
    def crossObs(self, x1, x2, y1, y2):
        obs = self.obstacles.copy()
        while (len(obs) > 0):
            rectang = obs.pop(0)
            # doing the interpolation to figure out if the object crosses any obstacles
            for i in range(0, 101):
                u = i / 100
                x = x1 * u + x2 * (1 - u)
                y = y1 * u + y2 * (1 - u)
            # collision 
                if rectang.collidepoint(x, y):
                    return True
        return False

    # connecting two nodes
    def connect(self, n1, n2):
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])
        if self.crossObs(x1, x2, y1, y2):
            self.remove_node(n2)
            return False
        else:
            self.add_edge(n1, n2)
            return True

    # creating a new node between the new node and the nnear found from nearest function
    # set a boundary of not eceeding a max of radius 35
    # dmax: it is the goal region radius so it can be changed
    def step(self, nnear, nrand, dmax=35):
        d = self.distance(nnear, nrand)
        # cheching if it is bigger than max or not
        if d > dmax:
            u = dmax / d
            (xnear, ynear) = (self.x[nnear], self.y[nnear])
            (xrand, yrand) = (self.x[nrand], self.y[nrand])
            # distance calculation 
            (px, py) = (xrand - xnear, yrand - ynear)
            theta = math.atan2(py, px)
            # coordinates of the new point (x1 = x + ncos(theta), y1 = y + nsin(theta))
            (x, y) = (int(xnear + dmax * math.cos(theta)),
                      int(ynear + dmax * math.sin(theta)))
            self.remove_node(nrand)
            # check if the new calculated node is withing the goal region of radius 35
            if abs(x - self.goal[0]) <= dmax and abs(y - self.goal[1]) <= dmax:
                self.add_node(nrand, self.goal[0], self.goal[1])
                self.goalstate = nrand
                self.goalFlag = True
            else:
                self.add_node(nrand, x, y)

    # finding the bias for the expansion in the direction of the goal (end point)
    def bias(self, ngoal):
        n = self.number_of_nodes()
        self.add_node(n, ngoal[0], ngoal[1])
        nnear = self.nearest(n)
        self.step(nnear, n)
        self.connect(nnear, n)
        return self.x, self.y, self.parent

    # random expansion of the tree
    def expand(self):
        n = self.number_of_nodes()
        x, y = self.sample_envir()
        self.add_node(n, x, y)
        if self.isFree():
            xnearest = self.nearest(n)
            self.step(xnearest, n)
            self.connect(xnearest, n)
        return self.x, self.y, self.parent

    # patj to the end point
    def path_to_goal(self):
        if self.goalFlag:
            self.path = []
            self.path.append(self.goalstate)
            newpos = self.parent[self.goalstate]
            while (newpos != 0):
                self.path.append(newpos)
                newpos = self.parent[newpos]
            self.path.append(0)
        return self.goalFlag

    # saving and returning path coordinates
    def getPathCoords(self):
        pathCoords = []
        for node in self.path:
            x, y = (self.x[node], self.y[node])
            pathCoords.append((x, y))
        return pathCoords


