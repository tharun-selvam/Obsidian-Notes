- [Video](https://www.youtube.com/watch?v=1f_m5aJFIj4)
- [Repo](https://github.com/lukovicaleksa/grid-mapping-in-ROS/blob/main/papers/Building%20an%20Efficient%20Occupancy%20Grid%20Map%20Based%20on%20Lidar%20Data.pdf)
- [Bresenham's line algorithm](https://www.youtube.com/watch?v=h3gDB89h0os)
# Repo Notes
- Bresenham's Line Algorithm is a computer graphics algorithm used for drawing straight lines on a raster display, which is a grid of pixels. Given two-coordinates, it returns the points to be drawn.
## Messagehandler.py
- Has two stack that receives Odom and LaserScan.
- Returns the latest two respective messages from the stack.
