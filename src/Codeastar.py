#!/usr/bin/env python

from heapq import heappush, heappop
import math, time
import roslib
import rospy
from std_msgs.msg import Int32
import os
from   std_msgs.msg import Int32MultiArray

xA=0
yA=0
xB=0
yB=0
xO=0
yO=0

class node:
    xPos = 0
    yPos = 0
    G = 0
    F = 0
    def __init__(self, xPos, yPos, G, F):
        self.xPos = xPos
        self.yPos = yPos
        self.G = G
        self.F = F
        
    def __lt__(self, other):
        return self.F < other.F
    def updateF(self, xEnd, yEnd):
        self.F = self.G + self.H(xEnd, yEnd) * 10
    def H(self, xEnd, yEnd):
        xd = xEnd - self.xPos
        yd = yEnd - self.yPos
        
        d = math.sqrt(xd*xd + yd*yd)
        
        return(d)
    
    def updateG(self, dirs, d):
        if dirs == 8 and d % 2 != 0:
            self.G = self.G + 14
        else:
            self.G = self.G +10
    
def pathPlanner(the_map, n, m, dirs, deltX, deltaY, xA, yA, xB, yB):
    closed_nodes = []
    open_nodes = []
    parent_dir = []
    row = [0] * n
    for i in range(m):
        closed_nodes.append(list(row))
        open_nodes.append(list(row))
        parent_dir.append(list(row))
        
    pq = [[], []]
    pqi = 0
        
    nA = node(xA, yA, 0, 0)
    nA.updateF(xB, yB)
    heappush (pq[pqi], nA)
    open_nodes[yA][xA] = nA.F
        
    while len(pq[pqi]) > 0:
            
        n1 = pq[pqi][0]
        nA = node(n1.xPos, n1.yPos, n1.G, n1.F)
        x = nA.xPos
        y = nA.yPos
        heappop(pq[pqi])
        open_nodes[y][x] = 0
        closed_nodes[y][x] = 1
            
                
        if x == xB and y ==  yB:
            path = ''
            
            while not (x == xA and y == yA):
                j = parent_dir[y][x]
                c = str((j + dirs/2) % dirs)
                path = c + path
                x = x + dx[j]
                y = y + dy[j]
                
            return path
            
        for i in range(dirs):
            xdx = x + dx[i]
            ydy = y + dy[i]
                
            if not (xdx < 0 or xdx > n-1 or ydy < 0 or ydy > m-1 or the_map[ydy][xdx] == 1
                    or closed_nodes[ydy][xdx] == 1):
                    
                m0 = node(xdx, ydy, nA.G, nA.F)
                m0.updateG(dirs, i)
                m0.updateF(xB, yB)
                    
                if open_nodes[ydy][xdx] == 0:
                    open_nodes[ydy][xdx] = m0.F
                    heappush(pq[pqi], m0)
                    parent_dir[ydy][xdx] = (i + dirs/2) % dirs
                        
                elif open_nodes[ydy][xdx] > m0.F:
                    open_nodes[ydy][xdx] = m0.F
                    parent_dir[ydy][xdx] = (i + dirs/2) % dirs
                        
                    while not (pq[pqi][0].xPos == xdx and pq[pqi][0].yPos == ydy):
                        heappush(pq[1-pqi], pq[pqi][0])
                        heappop(pq[pqi])
                    heappop(pq[pqi])
                        
                    if len(pq[pqi]) > len(pq[1-pqi]):
                        pqi = 1 - pqi
                    while len(pq[pqi]) > 0:
                        heappush(pq[1-pqi], pq[pqi][0])
                        heappop(pq[pqi])
                    pqi = 1 - pqi
                    heappush(pq[pqi], m0)
                        
    return 'hobba'





def callback2(data):
	global xA,yA
	xA=data.data[0]
	yA=data.data[1]

def callback1(data):
	global xR1,yR1 
	xR1=data.data[0]
	yR1=data.data[1]
	map_()

def callback3(data):
	global xB,yB
	xB=data.data[0]
	yB=data.data[1]
	


	
def listener():

	rospy.init_node('map')
	rospy.Subscriber('robot1',Int32MultiArray,callback1)
	rospy.Subscriber('robot2',Int32MultiArray,callback2)
	rospy.Subscriber('formation2',Int32MultiArray,callback3)

def map_():

	global dx,dy
	dirs = 8
	dx = [1, 1, 0, -1, -1, -1, 0, 1]
	dy = [0, 1, 1, 1, 0, -1, -1, -1]

	n=18
	m=14
	the_map = []
	row = [0] *n
	for i in range(m):
	    the_map.append(list(row))
	    
	#xA = xA
	#yA = yA
	#xB = xB
	#yB = yB
	the_map[yR1][xR1] = 1


	print 'Map size (X,Y):', n, m
	print 'Start:', xA, yA
	print 'Finish', xB, yB
	#print 'Obstacles', xO, yO

	t = time.time()
	route=pathPlanner(the_map, n, m, dirs, dx, dy, xA, yA, xB, yB)

	print 'Time=', time.time() - t
	print 'Route'
	print route

	if len(route) > 0:
	    x = xA
	    y = yA
	    the_map[y][x] = 2
	    for i in range(len(route)):
		j = int(route[i])
		x += dx[j]
		y += dy[j]
		the_map[y][x] = 3
	    the_map[y][x] = 4

	# display the map with the route added
	print 'Map:'
	for y in range(m):
	    for x in range(n):
		xy = the_map[y][x]
		if xy == 0:
		    print '.', # space
		elif xy == 1:
		    print 'I', # obstacle
		elif xy == 2:
		    print 'S', # start
		elif xy == 3:
		    print 'R', # route
		elif xy == 4:
		    print 'F', # finish
	    print
	os.system('clear')	    
	

if __name__ == '__main__':

	while not rospy.is_shutdown():
	
		listener()
		
		#raw_input('Press Enter...')
		rospy.spin()
