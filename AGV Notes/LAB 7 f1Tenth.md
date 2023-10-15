# RRT Algorithm
- [Video for implementation](https://www.youtube.com/watch?v=OXikozpLFGo)


# Notes from the slides
- We use RRT as a local planner. We use occupancy grid to expnad a tree to a goal point. 
- Then once we find the path, we have to form a spline out of the points found.
- Also need a collision checking function in the spline.
- We use a **clothoid** spline:
	- Constant rate of curvature change
	- Smothness and continuity beneficial for avoiding sudden changes
