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
    outputfile.write('akhlut\n')
    outputfile.write('5nbod7qe4d7d3vdvl6t9klkhh\n')
    for line in inputfile:
        answerstring = ''
        logging.info('Problem: '+str(count))
        if count in problems:
            answerstring+=str(count)+':'
            solution = instance(line)
            logging.info(str(solution))
            for robot_path in [path for path in solution if len(path) > 1][:-1]:
                for node in robot_path[:-1]:
                    answerstring+='('+str(node.x)+','+str(node.y)+'),'
                answerstring+='('+str(robot_path[-1].x)+','+str(robot_path[-1].y)+');'
            last_path = [path for path in solution if len(path) > 1][-1]
            for node in last_path[:-1]:
                answerstring+='('+str(node.x)+','+str(node.y)+'),'
            answerstring+='('+str(last_path[-1].x)+','+str(last_path[-1].y)+')'
            outputfile.write(answerstring+'\n')
        count+=1
    outputfile.close()
        
def instance(line):
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
    shortest_paths = [[[] for robot in robots] for robot in robots]
    logging.info("GENERATING SHORTEST PATHS...")
    pool = multiprocessing.Pool()
    path_count = 0
    for i in range(len(robots)):
        partial_sp_function = partial(a_star_algorithm,nodes[i],nodes,edges)
        shortest_paths[i] = pool.map(partial_sp_function,nodes[:len(robots)])
         # for j in range(len(robots)):
         #     if i <= j:
         #         continue
         #     n1 = nodes[i]
         #     n2 = nodes[j]
         #     shortest_path = a_star_algorithm(n2,nodes,edges,n1)
         #     shortest_paths[i][j]=shortest_path
         #     shortest_paths[j][i]=list(reversed(shortest_path))
         #     path_count+=2
                                     
    logging.info('GENERATED ['+str(path_count)+'] PATHS')
    logging.info('TRAVERSING...')
    pp = pprint.PrettyPrinter(indent=4)
    final_robot_paths = traverse(nodes[:len(robots)],shortest_paths)
    
    #PLOT:
    # logging.info('PLOTTING')
#     fig = plt.figure(1, figsize=(10,10), dpi=90)

#     ax = fig.add_subplot(111)
#     ax1= fig.add_subplot(111)
#     ax2 = fig.add_subplot(111)
#     ax3 = fig.add_subplot(111)
#     ax =  plotObstacles(obstacles,ax)
#     ax1 = plotRobots(robots,ax1)
# #    ax2 = plotShortestPaths(robots,nodes,shortest_paths,ax2)
#     ax3 = plotFinalRobotPaths(final_robot_paths,ax3)
#     plt.grid(b=True, which='both', color='0.65',linestyle='-')
#    # plt.show()
    return final_robot_paths
    

def traverse(robots,shortest_paths):
    awake = []
    not_awake = robots[1:]
    paths = [[] for robot in robots]
    logging.info('len paths: '+str(len(paths)))
    paths[0].append(robots[0])
    transit = [sys.maxsize for robot in robots]
    
    awake.append(robots[0])
    
    while len(not_awake) > 0:
        #logging.info('NEW OUTER ITERATION')
        new_awake = []
        has_not_claimed = Queue()
        for node in awake:
            has_not_claimed.put(node) 
        claims = {} #key is robot being claimed, value: (claiming robot, distance)
        #while not has_not_claimed.empty() and len(claims.keys()) < len(not_awake):
        min_transit = min([t for t in transit])
        for i in range(len(transit)):
            if transit[i]>0:
                transit[i] -= min_transit
        while True:
            prev_claims = claims.copy()
            while not has_not_claimed.empty():
                #logging.info('NEW INNER ITERATION')
                claim_node = has_not_claimed.get()
                #logging.info('CLAIM NODE: '+str(claim_node))
                claims,has_not_claimed = getMinPath(claim_node,paths[claim_node.index][-1],robots,shortest_paths,claims,has_not_claimed,awake,transit)

            if prev_claims == claims:
                break

        for dest in claims.keys():
            source = claims[dest][0]
            transit[source.index]=claims[dest][1]
            logging.info('transit[source.index]: '+str(claims[dest][1]))
            transit[dest.index] = transit[source.index]
            source_loc = paths[source.index][-1]
            path = shortest_paths[source_loc.index][dest.index]
            new_awake.append(dest)
            paths[source.index].extend(path[1:])
            paths[dest.index]+=[dest]
       
        logging.info(str(transit))
        awake.extend(new_awake)
        not_awake = [node for node in not_awake if node not in awake]
    return paths

def getMinPath(node,location,robots,paths,claims,has_not_claimed,awake,transit):
    #logging.info('Get Min Path, Robot: '+str(node)+' at Location: '+str(location))
    index = location.index
    minimum = sys.maxsize
    minipath = None
    destination = None
    paths = [path for path in paths[index]]
    sorted_paths = [(p,cost(p)) for p in sorted(paths,key=lambda x:cost(x))]
    distance_to_go = transit[node.index]
    for (path,this_cost) in sorted_paths:
        this_cost-=distance_to_go
        destination = path[-1]
        #logging.info('GET MIN PATH FOR LOOP')
        #logging.info('destination: '+str(destination)+' cost='+str(this_cost))
        #logging.info('CURRENT CLAIMS: '+str(claims))
        if destination not in awake:
            if destination in claims.keys() and this_cost<claims[destination][1]:
                old_claim = claims[destination]
                new_claim = (node,this_cost)
                logging.info('NODE '+str(node)+' HAS CLAIMED: '+str(destination)+' PREVIOUSLY OWNED BY '+str(old_claim))
                logging.info('NEW CLAIM: '+str(destination)+' by '+str(new_claim))
                has_not_claimed.put(old_claim[0]) #add current claimant to has not claimed
                claims[destination] = new_claim
                return claims,has_not_claimed
            elif destination not in claims.keys():
                new_claim = (node,this_cost)
                claims[destination]=new_claim
                logging.info(str(node)+' HAS CLAIMED: '+str(destination)+' PREVIOUSLY UNCLAIMED')
                logging.info('NEW CLAIM: '+str(new_claim))
                return claims,has_not_claimed
#    has_not_claimed.put(node)
    return claims,has_not_claimed
                
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
    
def a_star_algorithm(goal,nodes,edges,start):
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
            xs+=[node.x]
            ys+=[node.y]
        ax3.plot(xs,ys)
    return ax3
#args = inputfile, outputfile
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s %(message)s')
main(sys.argv[1:])
