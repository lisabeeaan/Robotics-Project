#!/usr/bin/env python
# coding: utf-8

# In[24]:


import random
import heapq
from PIL import Image
import numpy as np
import copy

#object to hold the data for one point or area in the occupancy map
class Node:
    def __init__(self,point,neighbours):
        arr=point.split(",")
        self.x=int(arr[0])
        self.y=int(arr[1])
        self.val=254
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
    goal.h=((start.x - goal.x)**2 + (start.y - goal.y)**2) ** 0.5
    goal.g=0
    start.h=goal.g
    start.g=goal.h
    return goal,start
    
#function to load the pgm file of the map and puts it in a 2d grid  
def loadpgm(filename):
    grid = np.array(Image.open(filename))
    return grid

#converts all the points in the grid to nodes to make it easier to use
def converttonodes(gridmap):
    cols=len(gridmap[0])
    rows=len(gridmap)
    obstacles=[]
    mapnodes=[]
    for y in range(rows):
        for x in range(cols):
            stri=str(x)+","+str(y)
            currnode=Node(stri,[])
            if(gridmap[y][x]!=254):
                obstacles.append(currnode)
            #else:
               # print(currnode.x,",",currnode.y)
            mapnodes.append(currnode)
            currnode.val=gridmap[y][x]
            currnode.x=x
            currnode.y=y
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
        if (x<cols and x>-1 and y>-1 and y<rows):
            if(gridmap[int(y)][int(x)]!=254):
                return False
        t+=0.01
    return True


                    
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
                #print(current.x,",",current.y)
                current = current.parent
            path.reverse()
            break
        else:
            
            for neigh in node.neighbours:
                newg = node.g + 1
                if (neigh not in visited or newg < neigh.g):
                    neigh.g = newg
                    neigh.h = ((neigh.x - goalnode.x)**2 + (neigh.y - goalnode.y)**2) ** 0.5
                    neigh.parent = node
                    heapq.heappush(queue, (neigh.g + neigh.h, neigh.h, neigh.g, neigh))
    #print("WE ARE FINISHED IN ASTAR")
    return path
    
#conects the nodes in the graph that are close to each other with no obstacles in between them to build the graph 
def findneighbours(gridmap,sample,samples,rad,obs):
    times=0
    for sam in samples:
        dist=((sample.x-sam.x)**2+(sample.y-sam.y)**2)**0.5
        if dist==rad or dist<rad:
            if sam not in sample.neighbours or sample not in sam.neighbours:
                if (isline(gridmap,sample,sam,obs) and sample!=sam):
                    #if times==0:
                        #print("we are finding neighbour for ",sample.x,",",sample.y)
                        #times+=1
                    sample.neighbours.append(sam)
                    sam.neighbours.append(sample)
                    
#learning phase which finds samples , saves the sample graph and then is ready for reuse to avoid multiple sampling 
def PRMphase1(obs,gridmap):
    samples=[]
    #print("we are in prm phase1")
    maxx=len(gridmap[0])
    maxy=len(gridmap)
    while (len(samples)<1000):
        xval=random.randint(0,maxx-1)
        yval=random.randint(0,maxy-1)
        string=str(int(xval))+","+str(int(yval))
        node=Node(string,[])
        node.val=gridmap[yval][xval]
        if(node.val==254):
            samples.append(node)
    #print("len of samples:",len(samples))
    for sample in samples:
        findneighbours(gridmap,sample,samples,maxx/24,obs)
    #print("WE ARE FINISHED IN PRM SAMPLING")
    return samples
    
#the query phase where the path from start to goal is found 
def PRMphase2(gridmap,obs,samples, goal,start):
    #print("we are phase 2")
    maxx=len(gridmap[0])
    tempsamples = samples.copy()
    findneighbours(gridmap,goal,tempsamples,maxx/3,obs)
    findneighbours(gridmap,start,tempsamples,maxx/3,obs)
    path=Astar(start, goal)
    return path 

def converttoworld(path):
    newpath = []
    for i in path:
        x = i.x * 0.1 + (-10)
        y = i.y * 0.1 + (-10)
        newpath.append((x, y))
    return newpath

#load the pgm into a 2D grid 
gridmap=loadpgm("my_map.pgm")
#gridmap=[row for row in gridmap if (len(row)>0)]
#converts entire grid into nodes and obstacles 
mapnodes,obstacles=converttonodes(gridmap)
#initiates the first phase of PRM
samples=PRMphase1(obstacles,gridmap)
str1=input("ENTER START")# put robots location 
str2=""
while(str2!="-1"):
    str2=input("ENTER GOAL")
    if(str2=="-1"):
        break
    #initialise goal and start str to nodes 
    goal,start=goalstartinit(str2,str1)
    path= PRMphase2(gridmap,obstacles,samples, goal,start)
    worldpath =converttoworld(path)
    str1=str(goal.x)+","+str(goal.y)
    for p in worldpath:
       outp = str(p[0]) + "," + str(p[1])
       print(outp)


# # print("hello")

# In[ ]:




