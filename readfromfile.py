import Queue as Q
import sys
from shapely.geometry import Polygon, LinearRing, LineString
import logging
import matplotlib.pyplot as plt
import numpy as np
from descartes import PolygonPatch

def main(argv):
    logging.info("main")
    inputfile=open(argv[0],"r")
    outputfile=argv[1]
    count = 1
    problems = eval(argv[2])
    for line in inputfile:
        logging.info('Problem: '+str(count))
        if count in problems:
            instance(line)
        count+=1
        
def instance(line):
    fig = plt.figure(1, figsize=(10,10), dpi=90)
    ax = fig.add_subplot(111)
    ax1= fig.add_subplot(111)
    ax2= fig.add_subplot(111)
    
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
    nodes += generateNodes(robots,'robot')
    obstacle_vertices = []
    for obstacle in obstacles:
        vertices = obstacle['coords']
        for vertex in vertices:
            obstacle_vertices+=[{'x':vertex[0],'y':vertex[1]}]
    nodes += generateNodes(obstacle_vertices,'vertex')

    logging.info('NUM NODES: ['+str(len(nodes))+']')
    logging.info('GENERATING EDGES...')
    
    #CREATE EDGES:
    edges = generateEdges(nodes,obstacles)
    logging.info('GENERATED EDGES')

    #SHORTEST PATHS BETWEEN ROBOTS:
    shortest_paths =[[({},{}) for i in range(len(robots))] for j in range(len(robots))]
    for i in range(len(robots)):
        for j in range(len(robots)):
            if j>= i:
                shortest_paths[i][j] = a_star_algorithm(nodes,edges,nodes[i],nodes[j])
    for i in range(len(robots)):
        for j in range(len(robots)):
            logging.info("SHORTEST PATH BETWEEN "+str(i)+" AND "+str(j)+" : ["+str(shortest_paths[i][j][0])+"]")
    
    logging.info('PLOTTING')
    
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

    ax.set_xticks(major_ticks)                                                       
    ax.set_xticks(minor_ticks, minor=True)                                           
    ax.set_yticks(major_ticks)                                                       
    ax.set_yticks(minor_ticks, minor=True)   

    for i in range(len(nodes)):
        n1=nodes[i]
        for j in range(len(nodes)):
            if j > i:
                n2 = nodes[j]
                if edges[i][j] > 0:
                    xs = [n1['x'],n2['x']]
                    ys = [n1['y'],n2['y']]
                    ax2.plot(xs,ys)
    
    plt.grid(b=True, which='both', color='0.65',linestyle='-')
    
    plt.show()

def distance(n1,n2):
    return (abs(n2['x']-n1['x'])+abs(n2['y']+n1['y']))
    
def a_star_algorithm(nodes,edges,start,goal):
    queue = Q.PriorityQueue()
    queue.put(start,0)
    came_from = {}
    cost_so_far = {}
    came_from[(start['x'],start['y'])] = None
    cost_so_far[(start['x'],start['y'])] = 0
    
    while not queue.empty():
        current = queue.get()
        
        if current == goal:
            break
        
        for next in neighbors(current,nodes,edges):
            g = cost_so_far[(current['x'],current['y'])] + distance(current, next)
            if (next['x'],next['y']) not in cost_so_far or g < cost_so_far[(next['x'],next['y'])]:
                cost_so_far[(next['x'],next['y'])] = g
                f = g + distance(goal, next)
                queue.put(next, f)
                came_from[(next['x'],next['y'])] = current
    
    return came_from, cost_so_far

def neighbors(node,nodes,edges):
    neighbors = []
    i = nodes.index(node)
    for j in range(len(nodes)):
        weight = edges[i][j]
        if weight > 0:
            neighbors+=[nodes[j]]
    return neighbors
            
        
def generateNodes(objects,desc):
    nodes = []
    for obj in objects:
        nodes+=[{'x':obj['x'],'y':obj['y'],'desc':desc}]
    return nodes

def generateEdges(nodes,obstacles):
    edges = [[-1 for i in range(len(nodes))] for j in range(len(nodes))]
    for i in range(len(nodes)):
        n1=nodes[i]
        for j in range(len(nodes)):
            if j>=i:
                n2 = nodes[j]
                if LOS(n1,n2,obstacles):
                    edges[i][j] = abs(n2['x']-n1['x'])+abs(n2['y']-n1['y'])
                else:
                    edges[i][j] = -1
                edges[j][i] = edges[i][j]
    return edges

## LINE OF SIGHT
def LOS(n1,n2,obstacles):
    p1 = (n1['x'],n1['y'])
    p2 = (n2['x'],n2['y'])
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
