# RRT Algorithm
- [Video for implementation](https://www.youtube.com/watch?v=OXikozpLFGo)


# Notes from the slides
- We use RRT as a local planner. We use occupancy grid to expnad a tree to a goal point. 
- Then once we find the path, we have to form a spline out of the points found.
- Also need a collision checking function in the spline.
- We use a **clothoid** spline:
	- Constant rate of curvature change
	- Smothness and continuity beneficial for avoiding sudden changes


```python
from PIL import Image, ImageOps

import numpy as np

import matplotlib.pyplot as plt

from matplotlib.pyplot import rcParams

import random

  

# img = Image.open('cspace.png')

  

# img = ImageOps.grayscale(img)

  

# np_img = np.array(img)

# np_img = ~np_img # invert B&W

# np_img[np_img > 0] = 1

# plt.set_cmap('binary')

# plt.imshow(np_img)

  

#Save Image

# np.save('cspace.npy, np_img')

  

#Read Image

# grid = np.load("cspace.npy")

# plt.imshow(grid, cmap="binary")

# plt.tight_layout()

# plt.show()

  

#Settings of Python plots

np.set_printoptions(precision=3, suppress=True)

rcParams['font.family'] = 'sans-serif'

rcParams['font.sans-serif'] = ['Tahoma']

plt.rcParams['font.size'] = 22

  
  

#treeNode class

  

class treeNode():

def __init__(self, locationX, locationY): #X location

self.locationX = locationX #Y Location

self.locationY = locationY #children list

self.children = [] #parent node reference

self.parent = None

  

#RRT Algorithm class

class RRTAlgorithm():

def __init__(self, start, goal, numIterations, grid, stepSize):

self.randomTree = treeNode(start[0], start[1]) #The RRT (root position)

self.goal = treeNode(goal[0], goal[1]) #goal position

self.nearestNode = None #nearest node

self.iterations = min(numIterations, 200) #number of iterations to run

self.grid = grid #the map

self.rho = stepSize #lenght of each branch

self.path_distance = 0 #total path distance

self.nearestDist = 10000 #distance to nearest node()

self.numWayPoints = 0 #number of waypoints

self.Waypoints = [] #the waypoints

#add the point to the nearest node and add goal when reached

def addChild(self, locationX, locationY):

if (locationX == self.goal.locationX):

#add the goal node to the children of the nearest node

self.nearestNode.children.append(self.goal)

self.goal.parent = self.nearestNode

else:

#add tempNode to children of nearest node

tempNode = treeNode(locationX, locationY)

self.nearestNode.children.append(tempNode)

tempNode.parent = self.nearestNode

  
  

#sample a random point within grid limits, <x, y> corresponds to grid <y, x>

def sampleAPoint(self):

x = random.randint(1, grid.shape[1])

y = random.randint(1, grid.shape[0])

point = np.array([x, y])

return point

  

#steer a distance stepsize form start to end location

def steerToPoint(self, locationStart, locationEnd):

offset = self.rho * self.unitVector(locationStart, locationEnd)

point = np.array([locationStart.locationX + offset[0], locationStart.locationY + offset[1]])

if point[0] >= grid.shape[1]:

point[0] = grid.shape[1]-1

if point[1] >= grid.shape[0]:

point[1] = grid.shape[0]-1

return point

  

#check if obstacle lies between the start node and end point of the edge

def isInObstacle(self, locationStart, locationEnd):

u_hat = self.unitVector(locationStart, locationEnd)

testPoint = np.array([0.0, 0.0])

for i in range(self.rho):

testPoint[0] = locationStart.locationX + i*u_hat[0]

testPoint[1] = locationStart.locationY + i*u_hat[1]

#check if testPoint lies within obstacle

if self.grid[round(testPoint[1]).astype(np.int64), round(testPoint[0]).astype(np.int64)] == 1:

return True

return False

  

#find unit vector between 2 points which form a vector

def unitVector(self, locationStart, locationEnd):

v = np.array([locationEnd[0] - locationStart.locationX, locationEnd[1] - locationStart.locationY])

u_hat = v/np.linalg.norm(v)

return u_hat

  

#find the nearest node from a given unconnected point (Euclidean distance)

def findNearest (self, root, point):

#return condition if root is NULL

#find distance between root and point

#if this is lower than the nearest Distacne, set this as the nearest node and update nearest distance

if not root:

return

dist = self.distance(root, point)

if dist <= self.nearestDist:

self.nearestNode = root

self.nearestDist = dist

  

#recursively call by iterating through the children

for child in root.children:

self.findNearest(child, point)

pass

  

#find euclidean distance b/w a node and an XY point

def distance(self, node1, point2):

dist = np.sqrt((node1.locaationX - point2[0])**2 + (node1.locaationY - point2[1])**2)

return dist

  

#check if the goal has been reached within step size

def goalFound(self, point):

if self.distance(self.goal, point) <= self.rho:

return True

  

#reset nearestNode and nearest Distacne

def resetNearestValues(self):

self.nearestNode = None

self.nearestDist = 10000

  

#trace the path from goal to start

def retraceRRTPath(self, goal):

if goal.locationX == self.randomTree.locationX:

return

#insert currentPoint to the Waypoints array from the beginning

self.numWayPoints += 1

currentPoint = np.array([goal.locationX, goal.locationY])

self.Waypoints.insert(0, currentPoint)

self.path_distance += self.rho

self.retraceRRTPath(goal.parent)

  

#end of class method definitions----------------------------------------------------------------#

  

#load the grid, set start and goal <x, y> positions, number of iterations, step size

grid = np.load( 'cspace.npy')

start = np.array ([100.0, 100.0])

goal = np.array ([700.0, 250.0])

numIterations = 200

stepSize = 50

goalRegion = plt.Circle((goal[0], goal [1]), stepSize, color='b', fill = False)

  

fig = plt.figure("RRT Algorithm")

plt.imshow (grid, cmap= 'binary')

plt.plot (start[0], start[1], 'ro')

plt.plot (goal[0], goal[1], 'bo')

ax = fig.gca()

ax.add_patch(goalRegion)

plt.xlabel('X-axis $(m)$')

plt.ylabel('Y-axis $(m)$')

  

#Begin

rrt = RRTAlgorithm(start, goal, numIterations, grid, stepSize)

  

for i in range(rrt.iterations) :

#Reset nearest values

rrt.resetNearestValues()

print("Iteration: ",i)

#algorithm begins here

point = rrt.sampleAPoint()

rrt.findNearest(rrt.randomTree, point)

new = rrt.steerToPoint(rrt.nearestNode, point)

bool = rrt.isInObstacle(rrt.nearestNode, new)

if (bool == False):

rrt.addChild(new[0], new[1])

plt.pause(0.10)

plt.plot([rrt.nearestNode.locationX, new[0]], [rrt.nearestNode.locationY, new[1]], 'go', linestyle="--")

#if goal found, append to path

if (rrt.goalfound(new)):

rrt.addchild(goal[0],goal[1])

print("Goal found!")

break

  

#trace back the path returned, and add start to waypoints

rrt.retraceRRTPath(rrt.goal)

rrt.Waypoints.insert(0, start)

print("Number of waypoints:", rrt.numWaypoints)

print("Path Distance (m): ", rrt.path_distance)

print("Waypoints: ", rrt.Waypoints)

  

#plot waypoints

#plot the waypoints

for i in range(len(rrt.Waypoints) - 1):

plt.plot([rrt.Waypoints[i][0], rrt.Waypoints[i+1][0]], [rrt.Waypoints[i][1], rrt.Waypoints[i+1][1]], 'ro', linestyle="--")

plt.pause(0.10)
```