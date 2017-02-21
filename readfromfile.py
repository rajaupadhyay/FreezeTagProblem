import math
from math import sqrt

import sys
from shapely.geometry import Polygon, LinearRing, LineString
import logging
import matplotlib.pyplot as plt
import numpy as np
from descartes import PolygonPatch
import pprint
import multiprocessing
from decimal import *

class Node:
    def __init__(self,index,x,y,desc):
        self.index = index
        self.x = x
        self.y = y
        self.desc = desc
    def __str__(self):
        return 'NODE <'+str(self.index)+'>: ('+str(self.x)+' , '+str(self.y)+')'
    def __repr__(self):
        return str(self)
    def __eq__(self,other):
        return self.index==other.index
    def __ne__(self,other):
        return not self.__eq__(other)
    def __hash__(self):
        return self.index
    
class Tree:
    def __init__(self):
        self.robots = []
    
def main(argv):
    logging.info("main")
    inputfile=open(argv[0],"r")
    outputfile=open(argv[1],"w")
    count = 1
    problems = eval(argv[2])
    for line in inputfile:
        logging.info('Problem: '+str(count))
        if count in problems:
            # solution = instance(line)
            # if solution is None:
            #     count+=1
            #     continue
            # answerstring = str(count) +': '
            # for node in solution[:-1]:
            #     answerstring += '('+str(node.x)+','+str(node.y)+'),'
            # answerstring += '('+str(solution[-1].x)+','+str(solution[-1].y)+')'
            # outputfile.write(answerstring)
            instance(line)
        count+=1
    outputfile.close()
        
def instance(line):
    fig = plt.figure(1, figsize=(10,10), dpi=90)
    ax = fig.add_subplot(111)
    ax1= fig.add_subplot(111)

    
    robots = []
    obstacles = []
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
    logging.info('NUMBER OF ROBOTS ['+str(len(robots))+']')
    logging.info('NUMBER OF OBSTACLES ['+str(len(obstacles))+']')

    #CREATE NODES:
    logging.info('GENERATING NODES...')
    nodes = []
    nodes += generateNodes(robots,'robot',0)
    obstacle_vertices = []
    for obstacle in obstacles:
        vertices = obstacle['coords']
        for vertex in vertices:
            obstacle_vertices+=[{'x':vertex[0],'y':vertex[1]}]
    nodes += generateNodes(obstacle_vertices,'vertex',len(robots))

    logging.info('NUM NODES: ['+str(len(nodes))+']')
    logging.info('GENERATING EDGES...')
    
    #CREATE EDGES:
    edges = generateEdges(nodes,obstacles)
    logging.info('GENERATED EDGES')

    #SHORTEST PATHS BETWEEN ROBOTS:
    shortest_paths = {}
    logging.info("GENERATING SHORTEST PATHS...")
    path_count = 0
    for i in range(len(robots)):
        for j in range(len(robots)):
            if i <= j:
                continue
            n1 = nodes[i]
            n2 = nodes[j]
            shortest_path = a_star_algorithm(nodes,edges,n2,n1)
            shortest_paths[(n1,n2)]=shortest_path
            shortest_paths[(n2,n1)]=list(reversed(shortest_path))
            path_count+=2
    logging.info('GENERATED ['+str(path_count)+'] PATHS')
#    logging.info('PLOTTING')
    logging.info('TRAVERSING...')
    pp = pprint.PrettyPrinter(indent=4)
    pp.pprint(traverse(nodes[:len(robots)],shortest_paths))
    
    #PLOT:
    for obstacle in obstacles:
        poly_patch = PolygonPatch(obstacle['polygon'])
        ax.add_patch(poly_patch)

    xs = []
    ys = []
    for robot in robots:
        xs +=[robot['x']]
        ys +=[robot['y']]

    ax1.scatter(xs,ys)
    ax.set_title('Polygon')
   
    major_ticks = np.arange(min(xs)-5, max(xs)+5, 1) 
    minor_ticks = np.arange(min(ys)-5, max(ys)+5, 1)
    path= fig.add_subplot(111)
    for i in range(len(robots)):
        n1 = nodes[i]
        for j in range(len(robots)):
            if i <= j:
                continue
            n2 = nodes[j]
            pathxs = []
            pathys = []
            shortest_path = shortest_paths[(n1,n2)]
            for node in shortest_path:
                pathxs.append(node.x)
                pathys.append(node.y)
            path.plot(pathxs,pathys)

    ax.set_xticks(major_ticks)                                                       
    ax.set_xticks(minor_ticks, minor=True)                                           
    ax.set_yticks(major_ticks)                                                       
    ax.set_yticks(minor_ticks, minor=True)   
    
    
    plt.grid(b=True, which='both', color='0.65',linestyle='-')
    
    plt.show()

def traverse(robots,shortest_paths):
    awake = []
    not_awake = robots[1:]
    paths = [[] for robot in robots]
    logging.info('len paths: '+str(len(paths)))
    paths[0].append(robots[0])
    
    awake.append(robots[0])
    while len(not_awake) > 0:
        logging.info('NEW OUTER ITERATION')
        new_awake = []
        has_not_claimed = awake[:]
        claims = {} #key is robot being claimed, value: (claiming robot, distance)
#        for robot in awake:
 #           if len(not_awake) == 0:
  #              break
        i = 0
        while i < len(has_not_claimed):
    #        logging.info(str(has_not_claimed))
            logging.info("i: "+str(i)+" len(has_not_claimed): "+str(len(has_not_claimed))+' has_not_claimed[i]: '+str(has_not_claimed[i]))
            claims,has_not_claimed,i = getMinPath(has_not_claimed[i],paths[has_not_claimed[i].index][-1],robots,shortest_paths,claims,has_not_claimed,awake,i)

        for dest in claims.keys():
            source = claims[dest][0]
            path = shortest_paths[(source,dest)]
            new_awake.append(dest)
            paths[source.index].extend(path)
            paths[dest.index]+=[dest]
            
        awake.extend(new_awake)
        not_awake = [node for node in not_awake if node not in awake]
    return paths
    logging.info(str(paths))

def getMinPath(node,location,robots,paths,claims,has_not_claimed,awake,i):
    logging.info('Get Min Path, Robot: '+str(node)+'\nLocation: '+str(location))
    index = location.index
    minimum = sys.maxsize
    minipath = None
    destination = None
    path_keys = [key for key in paths.keys() if key[0]==location]
    paths = [paths[key] for key in path_keys]
    sorted_paths = [(p,cost(p)) for p in sorted(paths,key=lambda x:cost(x))]
    
    for (path,this_cost) in sorted_paths:
        destination = path[-1]
        if destination not in awake:
            if destination in claims.keys() and this_cost<claims[destination][1]:
                claim = claims[destination]
                logging.info('NODE '+str(node)+' HAS CLAIMED: '+str(destination)+' PREVIOUSLY OWNED BY '+str(claim))
                new_claim = (node,this_cost)
                has_not_claimed.append(claim[0]) #add current claimant to has not claimed
                has_not_claimed.remove(node)
                claims[destination] = new_claim
                return claims,has_not_claimed,i
            elif destination not in claims.keys():
                claim = (node,this_cost)
                claims[destination]=claim
                logging.info(str(node)+' HAS CLAIMED: '+str(destination)+' PREVIOUSLY UNCLAIMED')
                has_not_claimed.remove(node)
                return claims,has_not_claimed,i

    i+=1
    return claims,has_not_claimed,i
                
    
                
            

def cost(path):
    cost = 0.0
    i = 0
    while i < len(path)-1:
        n1 = path[i]
        n2 = path[i+1]
        cost += distance(n1,n2)
        i+=1
    return cost
        
                   
def distance(n1,n2):
    return sqrt((n2.x-n1.x)**2 + (n2.y-n1.y)**2)
    
def a_star_algorithm(nodes,edges,start,goal):
    openSet = []
    gScore = {}
    fScore = {}
    closedSet = []
    cameFrom = {}
    
    for node in nodes:
        fScore[node] = sys.maxsize
        gScore[node] = sys.maxsize
    
    openSet.append(start)
    gScore[start] = 0
    fScore[start] = distance(start,goal)

    while len(openSet) > 0:
        current = (sorted([(node,fScore[node]) for node in openSet], key=lambda x:x[1])[0])[0]
        openSet.remove(current)
        if current == goal:
            return recreate_path(cameFrom,current)
        closedSet.append(current)
        for neighbor in neighbors(current,nodes,edges):
            if neighbor in closedSet:
                continue

            tent_gScore = gScore[current]+distance(current,neighbor)
            if neighbor not in openSet:
                openSet.append(neighbor)
            elif tent_gScore >= gScore[neighbor]:
                continue
            cameFrom[neighbor] = current
            gScore[neighbor]=tent_gScore
            fScore[neighbor]=gScore[neighbor]+distance(neighbor,goal)
    return None

def recreate_path(cameFrom,current):
    total_path = [current]
    while current in cameFrom.keys():
        current = cameFrom[current]
        total_path.append(current)
    return total_path
    
def neighbors(node,nodes,edges):
    neighbors = []
    i = nodes.index(node)
    for j in range(len(nodes)):
        weight = edges[i][j]
        if weight > 0:
            neighbors+=[nodes[j]]
    return neighbors
            
        
def generateNodes(objects,desc,start_id):
    i = start_id
    nodes = []
    for obj in objects:
        nodes+=[Node(i,obj['x'],obj['y'],desc)]
        i+=1
    return nodes

def generateEdges(nodes,obstacles):
    edges = [[-1 for i in range(len(nodes))] for j in range(len(nodes))]
    for i in range(len(nodes)):
        n1=nodes[i]
        for j in range(len(nodes)):
            if j>=i:
                n2 = nodes[j]
                if LOS(n1,n2,obstacles):
                    edges[i][j] = distance(n1,n2)
                else:
                    edges[i][j] = -1
                edges[j][i] = edges[i][j]
    return edges

## LINE OF SIGHT
def LOS(n1,n2,obstacles):
    p1 = (n1.x,n1.y)
    p2 = (n2.x,n2.y)
    line = LineString([p1,p2])
    obstructed = False
    for obstacle in obstacles:
        shape = obstacle['polygon']
        if line.crosses(shape) or shape.contains(line):
            obstructed = True
            break
    return not obstructed
                       
    
#args = inputfile, outputfile
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s %(message)s')
main(sys.argv[1:])
