import math
from math import sqrt
from queue import *
import sys
from shapely.geometry import Polygon, LinearRing, LineString
import logging
import matplotlib.pyplot as plt
import numpy as np
from descartes import PolygonPatch
import pprint
import multiprocessing
from decimal import *
from functools import partial

def main(argv):
    
    inputfile=open(argv[0],"r")
    pathsfile=open(argv[1],"r")
    numprob  =argv[2]
    robots = []
    obstacles = []
    for line in inputfile:
        problem = line.split(":")[0]
        if problem != numprob:
            continue
        line = line.split(":")[1]
        if "#" in line:
            x = line.split("#")
            robot_coordinates = eval(x[0])
            for coord_pair in robot_coordinates:
                robots+=[{'x':coord_pair[0],'y':coord_pair[1],'awake':False}]
            robots[0]['awake']=True
            #        logging.info(robots)
            
            shapes_str = x[1].split(";")
            for shape_str in shapes_str:
                shape = eval(shape_str)
                obstacles += [{'coords':shape,'polygon':Polygon(shape)}]

   #        logging.info(obstacles)

        else:
            robot_coordinates = eval(line)
            for coord_pair in robot_coordinates:
                robots+=[{'x':coord_pair[0],'y':coord_pair[1],'awake':False}]
            robots[0]['awake']=True
   #        logging.info(robots)
    
    fig = plt.figure(1, figsize=(10,10), dpi=90)
    ax = fig.add_subplot(111)
    ax1= fig.add_subplot(111)
    ax3 = fig.add_subplot(111)
    ax =  plotObstacles(obstacles,ax)
    ax1 = plotRobots(robots,ax1)
    shortest_paths = []
    for line in pathsfile.readlines()[2:]:
        problem = line.split(":")[0]
        print(problem)
        if problem != numprob:
            continue
        print(problem)
        line = line.split(':')[1]
        paths = line.split(';')
        for path in paths:
            coords = eval(path)
            print(path)
            shortest_paths.append(coords)
            
    


    
    ax3 = plotFinalRobotPaths(shortest_paths,ax3)
    plt.show()
        

def plotObstacles(obstacles,ax):
    for obstacle in obstacles:
        poly_patch = PolygonPatch(obstacle['polygon'])
        ax.add_patch(poly_patch)
    return ax

def plotRobots(robots,ax):
    xs = []
    ys = []
    for robot in robots:
        xs +=[robot['x']]
        ys +=[robot['y']]
    major_ticks = np.arange(min(xs)-5, max(xs)+5, 1) 
    minor_ticks = np.arange(min(ys)-5, max(ys)+5, 1)
    ax.set_xticks(major_ticks)                                                       
    ax.set_xticks(minor_ticks, minor=True)                                           
    ax.set_yticks(major_ticks)                                                       
    ax.set_yticks(minor_ticks, minor=True)   

    ax.scatter(xs,ys)
    return ax

def plotShortestPaths(robots,nodes,shortest_paths,path):
    for i in range(len(robots)):
        n1 = nodes[i]
        for j in range(len(robots)):
            if i <= j:
                continue
            n2 = nodes[j]
            pathxs = []
            pathys = []
            shortest_path = shortest_paths[i][j]
            for node in shortest_path:
                pathxs.append(node.x)
                pathys.append(node.y)
            path.plot(pathxs,pathys)
    return path

def plotFinalRobotPaths(paths,ax3):
    for path in paths:
        xs =[]
        ys =[]
        for node in path:
            xs+=[node[0]]
            ys+=[node[1]]
        ax3.plot(xs,ys)
    return ax3

main(sys.argv[1:])
