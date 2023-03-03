import os
import pygame
from RRT_base import RRTGraph
from RRT_base import RRTMap
import time


clear = lambda: os.system('cls')  # On Windows System
clear()

# defining the desired dimensions, number and size of the obstacles, the start and goal points
# in the main class
def main():
    dimensions = (600, 1000)
    start = (50, 50)
    goal = (512, 512)
    obsdim = 30
    obstnum = 50
    iteration = 0
    t1 = 0

    pygame.init()
    map = RRTMap(start, goal, dimensions, obsdim, obstnum)
    graph = RRTGraph(start, goal, dimensions, obsdim, obstnum)

    # making the obstacle
    obstacles=graph.makeobs()

    # drawing the map
    map.drawMap(obstacles)

    # # creating a random sample and adding it to the tree in order to test it
    # while(True):
    #     x, y = graph.sample_envir()
    #     n = graph.number_of_nodes()
    #     graph.add_node(n, x, y)
    #     graph.add_edge(n-1, n)
    #     x1, y1 = graph.x[n], graph.y[n]
    #     x2, y2 = graph.x[n-1], graph.y[n-1]
    #     if(graph.isFree()):
    #         pygame.draw.circle(map.map, map.Red, (graph.x[n], graph.y[n]), map.nodeRadius, map.nodeThickness)
    #         if not graph.crossObs(x1, x2, y1, y2):
    #             pygame.draw.line(map.map, map.Blue, (x1, y1), (x2, y2), map.edgeThickness)

    #     # to see the drawing objects we need to update the display
    #     pygame.display.update()
    #     pygame.event.clear()
    #     pygame.event.wait(0) # this will freeze the window until we press tab or move the mouse

    # bias and expansion check (500 is the limit of the scene)
    t1=time.time()
    while (not graph.path_to_goal()):
        # time.sleep(0.005)
        # elapsed=time.time()-t1
        # t1=time.time()
        #raise exception if timeout
        # if elapsed > 10:
        #     print('timeout re-initiating the calculations')
        #     raise

        if iteration % 10 == 0:
            X, Y, Parent = graph.bias(goal)
            pygame.draw.circle(map.map, map.grey, (X[-1], Y[-1]), map.nodeRad*2, 0)
            pygame.draw.line(map.map, map.Blue, (X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]),
                             map.edgeThickness)

        else:
            X, Y, Parent = graph.expand()
            pygame.draw.circle(map.map, map.grey, (X[-1], Y[-1]), map.nodeRad*2, 0)
            pygame.draw.line(map.map, map.Blue, (X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]),
                             map.edgeThickness)

        if iteration % 5 == 0:
            pygame.display.update()
        iteration += 1
    map.drawPath(graph.getPathCoords())
    pygame.display.update()
    pygame.event.clear()
    pygame.event.wait(0) # this will freeze the window until we press tab or move the mouse
        



if __name__ == '__main__':
    main()
