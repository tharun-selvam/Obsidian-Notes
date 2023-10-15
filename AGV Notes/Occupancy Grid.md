- [Video](https://www.youtube.com/watch?v=1f_m5aJFIj4)
- [Repo](https://github.com/lukovicaleksa/grid-mapping-in-ROS/blob/main/papers/Building%20an%20Efficient%20Occupancy%20Grid%20Map%20Based%20on%20Lidar%20Data.pdf)
- [Bresenham's line algorithm](https://www.youtube.com/watch?v=h3gDB89h0os)
# Repo Notes
- Bresenham's Line Algorithm is a computer graphics algorithm used for drawing straight lines on a raster display, which is a grid of pixels. Given two-coordinates, it returns the points to be drawn.
## Messagehandler.py
- Has two stack that receives Odom and LaserScan.
- Returns the latest two respective messages from the stack.

```python
#!/usr/bin/env python

  

import rclpy

from rclpy.node import Node

from sensor_msgs.msg import LaserScan

from nav_msgs.msg import Odometry

from tf2.transformations import euler_from_quaternion, quaternion_from_euler

  

import cv2

import numpy as np

import matplotlib.pyplot as plt

from time import perf_counter

  

#------------------bresenham-start----------------------------------------------------------------------

  

def bresenham(gridMap, x1, y1, x2, y2):

"""

Bresenham's line drawing algorithm - working for all 4 quadrants!

"""

# Output pixels

X_bres = []

Y_bres = []

  

x = x1

y = y1

delta_x = np.abs(x2 - x1)

delta_y = np.abs(y2 - y1)

s_x = np.sign(x2 - x1)

s_y = np.sign(y2 - y1)

  

if delta_y > delta_x:

  

delta_x, delta_y = delta_y, delta_x

interchange = True

  

else:

  

interchange = False

  

A = 2 * delta_y

B = 2 * (delta_y - delta_x)

E = 2 * delta_y - delta_x

  

# mark output pixels

X_bres.append(x)

Y_bres.append(y)

  

# point (x2,y2) must not be included

for i in range(1, delta_x):

  

if E < 0:

  

if interchange:

  

y += s_y

else:

  

x += s_x

  

E = E + A

  

else:

  

y += s_y

x += s_x

E = E + B

  

# mark output pixels

X_bres.append(x)

Y_bres.append(y)

  

return zip(X_bres, Y_bres)

  

#------------------bresenham-end----------------------------------------------------------------------

  
  

#------------------grid-map-start----------------------------------------------------------------------

  

TRESHOLD_P_FREE = 0.3

TRESHOLD_P_OCC = 0.6

  

def log_odds(p):

"""

Log odds ratio of p(x):

  

p(x)

l(x) = log ----------

1 - p(x)

  

"""

return np.log(p / (1 - p))

  
  

def retrieve_p(l):

"""

Retrieve p(x) from log odds ratio:

  

1

p(x) = 1 - ---------------

1 + exp(l(x))

  

"""

return 1 - 1 / (1 + np.exp(l))

  

class GridMap:

"""

Grid map

"""

def __init__(self, X_lim, Y_lim, resolution, p):

  

self.X_lim = X_lim

self.Y_lim = Y_lim

self.resolution = resolution

  

x = np.arange(start = X_lim[0], stop = X_lim[1] + resolution, step = resolution)

y = np.arange(start = Y_lim[0], stop = Y_lim[1] + resolution, step = resolution)

# probability matrix in log-odds scale:

self.l = np.full(shape = (len(x), len(y)), fill_value = log_odds(p))

  

def get_shape(self):

"""

Get dimensions

"""

return np.shape(self.l)

  

def calc_MLE(self):

"""

Calculate Maximum Likelihood estimate of the map

"""

for x in range(self.l.shape[0]):

  

for y in range(self.l.shape[1]):

  

# cell is free

if self.l[x][y] < log_odds(TRESHOLD_P_FREE):

  

self.l[x][y] = log_odds(0.01)

  

# cell is occupied

elif self.l[x][y] > log_odds(TRESHOLD_P_OCC):

  

self.l[x][y] = log_odds(0.99)

  

# cell state uncertain

else:

  

self.l[x][y] = log_odds(0.5)

  

def to_BGR_image(self):

"""

Transformation to BGR image format

"""

# grayscale image

gray_image = 1 - retrieve_p(self.l)

  

# repeat values of grayscale image among 3 axis to get BGR image

rgb_image = np.repeat(a = gray_image[:,:,np.newaxis],

repeats = 3,

axis = 2)

  

return rgb_image

  

def to_grayscale_image(self):

"""

Transformation to GRAYSCALE image format

"""

return 1 - retrieve_p(self.l)

  

def discretize(self, x_cont, y_cont):

"""

Discretize continious x and y

"""

x = int((x_cont - self.X_lim[0]) / self.resolution)

y = int((y_cont - self.Y_lim[0]) / self.resolution)

return (x,y)

  

def update(self, x, y, p):

"""

Update x and y coordinates in discretized grid map

"""

# update probability matrix using inverse sensor model

self.l[x][y] += log_odds(p)

  

def check_pixel(self, x, y):

"""

Check if pixel (x,y) is within the map bounds

"""

if x >= 0 and x < self.get_shape()[0] and y >= 0 and y < self.get_shape()[1]:

return True

  

else:

  

return False

  

def find_neighbours(self, x, y):

"""

Find neighbouring pixels to pixel (x,y)

"""

X_neighbours = []

Y_neighbours = []

  

if self.check_pixel(x + 1, y):

  

X_neighbours.append(x + 1)

Y_neighbours.append(y)

  

if self.check_pixel(x + 1, y + 1):

  

X_neighbours.append(x + 1)

Y_neighbours.append(y + 1)

  

if self.check_pixel(x + 1, y - 1):

  

X_neighbours.append(x + 1)

Y_neighbours.append(y - 1)

  

if self.check_pixel(x, y + 1):

  

X_neighbours.append(x)

Y_neighbours.append(y + 1)

  

if self.check_pixel(x, y - 1):

  

X_neighbours.append(x)

Y_neighbours.append(y - 1)

  

if self.check_pixel(x - 1, y):

  

X_neighbours.append(x - 1)

Y_neighbours.append(y)

  

if self.check_pixel(x - 1, y + 1):

  

X_neighbours.append(x - 1)

Y_neighbours.append(y + 1)

  

if self.check_pixel(x - 1, y - 1):

  

X_neighbours.append(x - 1)

Y_neighbours.append(y - 1)

  

return zip(X_neighbours, Y_neighbours)

  
  

def set_pixel_color(bgr_image, x, y, color):

"""

Set 'color' to the given pixel (x,y) on 'bgr_image'

"""

  

if x < 0 or y < 0 or x >= bgr_image.shape[0] or y >= bgr_image.shape[1]:

return

  

if color == 'BLUE':

  

bgr_image[x, y, 0] = 1.0

bgr_image[x, y, 1] = 0.0

bgr_image[x, y, 2] = 0.0

  

elif color == 'GREEN':

  

bgr_image[x, y, 0] = 0.0

bgr_image[x, y, 1] = 1.0

bgr_image[x, y, 2] = 0.0

  

elif color == 'RED':

  

bgr_image[x, y, 0] = 0.0

bgr_image[x, y, 1] = 0.0

bgr_image[x, y, 2] = 1.0

  
  

#------------------grid-map-end----------------------------------------------------------------------

  
  
  
  
  
  
  

#------------------utils-start----------------------------------------------------------------------

  

def lidar_scan(msgScan):

"""

Convert LaserScan msg to array

"""

distances = np.array([])

angles = np.array([])

information = np.array([])

  

for i in range(len(msgScan.ranges)):

# angle calculation

ang = i * msgScan.angle_increment

  

# distance calculation

if ( msgScan.ranges[i] > msgScan.range_max ):

dist = msgScan.range_max

elif ( msgScan.ranges[i] < msgScan.range_min ):

dist = msgScan.range_min

else:

dist = msgScan.ranges[i]

  

# smaller the distance, bigger the information (measurement is more confident)

inf = ((msgScan.range_max - dist) / msgScan.range_max) ** 2

  

distances = np.append(distances, dist)

angles = np.append(angles, ang)

information = np.append(information, inf)

  

# distances in [m], angles in [radians], information [0-1]

return ( distances, angles, information )

  
  

def lidar_scan_xy(distances, angles, x_odom, y_odom, theta_odom):

"""

Lidar measurements in X-Y plane

"""

distances_x = np.array([])

distances_y = np.array([])

  

for (dist, ang) in zip(distances, angles):

distances_x = np.append(distances_x, x_odom + dist * np.cos(ang + theta_odom))

distances_y = np.append(distances_y, y_odom + dist * np.sin(ang + theta_odom))

  

return (distances_x, distances_y)

  
  

def transform_orientation(orientation_q):

"""

Transform theta to [radians] from [quaternion orientation]

"""

orientation_list = [ orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

(roll, pitch, yaw) = euler_from_quaternion(orientation_list)

if yaw < 0:

yaw = 2 * np.pi + yaw # 0->360 degrees >> 0->2pi

return yaw

  
  

def get_odom_orientation(msgOdom):

""""

Get theta from Odometry msg in [radians]

"""

orientation_q = msgOdom.pose.pose.orientation

theta = transform_orientation(orientation_q)

return theta

  

def get_odom_position(msgOdom):

"""

Get (x,y) coordinates from Odometry msg in [m]

"""

x = msgOdom.pose.pose.position.x

y = msgOdom.pose.pose.position.y

return (x, y)

  
  
  

#------------------utils-end----------------------------------------------------------------------

  
  
  
  

def main(args=None):

  

try:

  

P_prior = 0.5 # Prior occupancy probability

P_occ = 0.9 # Probability that cell is occupied with total confidence

P_free = 0.3 # Probability that cell is free with total confidence

  

RESOLUTION = 0.05 # Grid resolution in [m]

map_x_lim = [-16, 17]

map_y_lim = [-8, 15]

  

MAP_NAME = 'levine_tharun'

  

# Init ROS Node

rclpy.init(args=args)

node = rclpy.create_node('ros2_gmapping_node')

rate = rclpy.Rate(10)

  

# Create grid map

gridMap = GridMap(X_lim = map_x_lim,

Y_lim = map_y_lim,

resolution = RESOLUTION,

p = P_prior)

# Init time

t_start = perf_counter()

sim_time = 0

step = 0

  

while rclpy.is_ok():

  

# Lidar measurements

msgScan = rclpy.wait_for_message(node, LaserScan, '/scan')

distances, angles, information = lidar_scan(msgScan) # distances in [m], angles in [radians]

  

# Odometry measurements

msgOdom = rclpy.wait_for_message(node, Odometry, '/odom')

x_odom, y_odom = get_odom_position(msgOdom) # x,y in [m]

theta_odom = get_odom_orientation(msgOdom) # theta in [radians]

  

# Lidar measurements in X-Y plane

distances_x, distances_y = lidar_scan_xy(distances, angles, x_odom, y_odom, theta_odom)

  

# x1 and y1 for Bresenham's algorithm

x1, y1 = gridMap.discretize(x_odom, y_odom)

  

# for BGR image of the grid map

X2 = []

Y2 = []

  

for (dist_x, dist_y, dist) in zip(distances_x, distances_y, distances):

  

# x2 and y2 for Bresenham's algorithm

x2, y2 = gridMap.discretize(dist_x, dist_y)

  

# draw a discrete line of free pixels, [robot position -> laser hit spot)

for (x_bres, y_bres) in bresenham(gridMap, x1, y1, x2, y2):

gridMap.update(x = x_bres, y = y_bres, p = P_free)

  

# mark laser hit spot as ocuppied (if exists)

if dist < msgScan.range_max:

gridMap.update(x = x2, y = y2, p = P_occ)

  

# for BGR image of the grid map

X2.append(x2)

Y2.append(y2)

  

# converting grid map to BGR image

bgr_image = gridMap.to_BGR_image()

  

# marking robot position with blue pixel value

set_pixel_color(bgr_image, x1, y1, 'BLUE')

# marking neighbouring pixels with blue pixel value

for (x, y) in gridMap.find_neighbours(x1, y1):

set_pixel_color(bgr_image, x, y, 'BLUE')

  

# marking laser hit spots with green value

for (x, y) in zip(X2,Y2):

set_pixel_color(bgr_image, x, y, 'GREEN')

  

resized_image = cv2.resize(src = bgr_image,

dsize = (500, 500),

interpolation = cv2.INTER_AREA)

  

rotated_image = cv2.rotate(src = resized_image,

rotateCode = cv2.ROTATE_90_COUNTERCLOCKWISE)

  

cv2.imshow("Grid map", rotated_image)

cv2.waitKey(1)

  

# Calculate step time in [s]

t_step = perf_counter()

step_time = t_step - t_start

sim_time += step_time

t_start = t_step

step += 1

  

print('Step %d ==> %d [ms]' % (step, step_time * 1000))

  

rate.sleep()

  

except rclpy.ROSInterruptException:

  

print('\r\nSIMULATION TERMINATED!')

print('\nSimulation time: %.2f [s]' % sim_time)

print('Average step time: %d [ms]' % (sim_time * 1000 / step))

print('Frames per second: %.1f' % (step / sim_time))

  

# Saving Grid Map

resized_image = cv2.resize(src = gridMap.to_BGR_image(),

dsize = (500, 500),

interpolation = cv2.INTER_AREA)

  

rotated_image = cv2.rotate(src = resized_image,

rotateCode = cv2.ROTATE_90_COUNTERCLOCKWISE)

  

flag_1 = cv2.imwrite(img = rotated_image * 255.0,

filename = MAPS_PATH + '/' + MAP_NAME + '_grid_map_TEST.png')

  

# Calculating Maximum likelihood estimate of the map

gridMap.calc_MLE()

  

# Saving MLE of the Grid Map

resized_image_MLE = cv2.resize(src = gridMap.to_BGR_image(),

dsize = (500, 500),

interpolation = cv2.INTER_AREA)

  

rotated_image_MLE = cv2.rotate(src = resized_image_MLE,

rotateCode = cv2.ROTATE_90_COUNTERCLOCKWISE)

  

flag_2 = cv2.imwrite(img = rotated_image_MLE * 255.0,

filename = MAPS_PATH + '/' + MAP_NAME + '_grid_map_TEST_mle.png')

  

if flag_1 and flag_2:

print('\nGrid map successfully saved!\n')

  

if cv2.waitKey(0) == 27:

cv2.destroyAllWindows()

  

pass

  

rclpy.shutdown()

  
  

if __name__ == '__main__':

main()
```