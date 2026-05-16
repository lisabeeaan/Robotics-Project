#!/usr/bin/env python
# coding: utf-8

# In[33]:


import random
import heapq

def prm_plan(map_yaml_path, start_str, goal_str):
    # Load YAML to get PGM filename
    import yaml
    with open(map_yaml_path, 'r') as f:
        map_data = yaml.safe_load(f)
    import os

    # YAML gives a relative path like "my_map.pgm"
    pgm_relative = map_data["image"]

    # Build absolute path based on the YAML file location
    yaml_dir = os.path.dirname(map_yaml_path)
    pgm_file = os.path.join(yaml_dir, pgm_relative)

    # Load the PGM map
    gridmap = loadpgm(pgm_file)
    gridmap = [row for row in gridmap if len(row) > 0]

    # Convert grid to nodes + obstacles
    mapnodes, obstacles = converttonodes(gridmap)

    # Convert start/goal strings to Node objects
    goal, start = goalstartinit(start_str, goal_str)

    # PRM sampling
    samples = PRMphase1(goal, start, obstacles, gridmap)

    # PRM query
    path_nodes = PRMphase2(gridmap, obstacles, samples, goal, start)

    # Convert Node objects to (x, y) waypoints
    waypoints = [(p.x, p.y) for p in path_nodes]

    return waypoints
            
#object to hold the data for one point or area in the occupancy map
class Node:
    def __init__(self,point,neighbours):
        arr=point.split(",")
        self.x=int(arr[0])
        self.y=int(arr[1])
        self.val=255
        self.neighbours=neighbours
        self.h=0
        self.g=0
        self.parent=None
        
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))
        
    def __lt__(self, other):
        return (self.x, self.y) < (other.x, other.y)

def goalstartinit(startstrlocation,goalstrlocation):
    
    goal=Node(goalstrlocation,[])
    start=Node(startstrlocation,[]) #going to be the robots current position 
    #the distances for both for astar 
    goal.h=(((goal.x-goal.x)**2 +(goal.y-goal.y)**2))**0.5;
    goal.g=(((start.x-goal.x)**2 +(goal.y-start.y)**2))**0.5;
    start.h=goal.g
    start.g=goal.h
    return goal,start
    
#function to load the pgm file of the map and puts it in a 2d grid  
def loadpgm(filename):
    with open(filename, 'r') as f:
        # Strip whitespace
        lines = [l.strip() for l in f.readlines()]

    # Remove blank lines
    lines = [l for l in lines if l != ""]

    # Remove comment lines
    lines = [l for l in lines if not l.startswith("#")]

    assert lines[0] == "P2"

    width, height = map(int, lines[1].split())
    max_val = int(lines[2])

    data = []
    for line in lines[3:]:
        data.extend([int(x) for x in line.split()])

    grid = []
    for i in range(height):
        row = data[i * width:(i + 1) * width]
        grid.append(row)

    return grid

#converts all the points in the grid to nodes to make it easier to use
def converttonodes(gridmap):
    cols=len(gridmap[0])
    rows=len(gridmap)
    obstacles=[]
    mapnodes=[]
    for i in range(rows):
        for j in range(cols):
            stri=str(j)+","+str(i)
            currnode=Node(stri,[])
            if(gridmap[i][j]!=255):
                obstacles.append(currnode)
            mapnodes.append(currnode)
            currnode.val=gridmap[i][j]
            currnode.x=i
            currnode.y=j
    return mapnodes,obstacles

#mini function to check if a point is a obstacle
def isobstacle(point,obs):
    if(point in obs):
        return True
    else:
        return False

#checks if we can draw a clear line from node to node without hitting an obstacle            
def isline(gridmap,point1,point2,obs):
    cols=len(gridmap[0])
    rows=len(gridmap)
    t=0.0
    leny=(point2.y-point1.y)
    lenx=(point2.x-point1.x)
    x = point1.x+t*lenx
    y = point1.y+t*leny
    
    while(t<=1):
        x = point1.x+t*lenx
        y = point1.y+t*leny
        if (x<rows and x>0 and y>0 and y<cols):
            xy= str(int(x))+","+str(int(y))
            xynode=Node(xy,[])
            if(isobstacle(xynode,obs)):
                return False
        t+=0.05
    return True

#conects the nodes in the graph that are close to each other with no obstacles in between them to build the graph 
def findneighbours(gridmap,sample,samples,rad,obs):
    for sam in samples:
        dist=((sample.x-sam.x)**2+(sample.y-sam.y)**2)**0.5
        if dist==rad or dist<rad:
            if sam not in sample.neighbours:
                if (isline(gridmap,sample,sam,obs) and sample!=sam):
                    sample.neighbours.append(sam)
                    sam.neighbours.append(sample)
                    
                    
#astart to find the shortest path from start to goal 
def Astar(startnode, goalnode):
    #print("WE ARE IN ASTAR")
    visited=set();
    path=[]
    queue=[]
    heapq.heappush(queue, ((startnode.h+startnode.g),startnode.h,startnode.g,startnode))
    while queue:
        cost,h,g,node= heapq.heappop(queue)
        if node in visited:
            continue
        visited.add(node)
        
        if(node.x==goalnode.x and node.y==goalnode.y):
            #print("THIS IS THE GOAL STATE" ,node.x,node.y)
            current = node
            while current is not None:
                path.append(current)
                current = current.parent
            path.reverse()
            break
        else:
            
            for neigh in node.neighbours:
                if (neigh not in visited):
                    neigh.g = node.g + 1
                    neigh.parent = node
                    heapq.heappush(queue, (neigh.g + neigh.h, neigh.h, neigh.g, neigh))
    #print("WE ARE FINISHED IN ASTAR")
    return path
    


#learning phase which finds samples , saves the sample graph and then is ready for reuse to avoid multiple sampling 
def PRMphase1(goal,start,obs,gridmap):
    samples=[]
    #print("we are in prm")
    maxx=len(gridmap[0])
    maxy=len(gridmap)
    while (len(samples)<200):
        xval=random.randint(0,maxx)
        yval=random.randint(0,maxy)
        string=str(int(xval))+","+str(int(yval))
        node=Node(string,[])
        if(isobstacle(node,obs)==False):
            samples.append(node)
    for sample in samples:
        findneighbours(gridmap,sample,samples,maxx/2,obs)
    #print("WE ARE FINISHED IN PRM SAMPLING")
    return samples
    
#the query phase where the path from start to goal is found 
def PRMphase2(gridmap,obs,samples, goal,start):
    maxx=len(gridmap[0])
    tempsamples = list(samples)
    tempsamples.append(goal)
    tempsamples.append(start)
    findneighbours(gridmap,goal,tempsamples,maxx/2,obs)
    findneighbours(gridmap,start,tempsamples,maxx/2,obs)
    for node in tempsamples:
        node.h=(((node.x-goal.x)**2 +(node.y-goal.y)**2))**0.5;
        node.g=(((node.x-start.x)**2 +(node.y-start.y)**2))**0.5;
    path=Astar(start, goal)
    return path 

if __name__ == "__main__":
    str2 = input("ENTER START")
    str1 = input("ENTER GOAL")
    goal, start = goalstartinit(str2, str1)

    gridmap = loadpgm("test_pgm.pgm")
    gridmap = [row for row in gridmap if (len(row) > 0)]

    mapnodes, obstacles = converttonodes(gridmap)

    samples = PRMphase1(goal, start, obstacles, gridmap)

    path = PRMphase2(gridmap, obstacles, samples, goal, start)
    for p in path:
        outp = str(p.x) + "," + str(p.y)
        print(outp)

# In[ ]:





# In[ ]:





# In[ ]:




