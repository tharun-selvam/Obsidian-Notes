- [Resource](https://www.youtube.com/watch?v=5nZc5iSr5is&list=RDCMUCt6Lag-vv25fTX3e11mVY1Q&start_radio=1&rv=5nZc5iSr5is&t=1446)

## Important Topics
- **/cmd_vel**: Receives the output of the Navigation Stack and transforms the commands into motor velocities.
- **/kobuki/laser/scan**: Provides the Laser readings to the Stack.  
    
- **/odom**: Provides the Odometry readings to the Stack.  
    
- **/tf**: Provides the Transformations to the Stack.

## Video 2
- Open RViz. 
- Add Laser and map components by specifiying the topic also.
1. Builidng the map.
	- Open teleop and navigate around manually to build the full map.
	- We have a package `gmapping` which is used to create the map
	-  `slam_gmapiing` node of this package allows us to create the map. Subscribes to laser and transform from the robot and builds an occupancy grid by publishing to the `/map` .
	- The `/map` topic uses a message type `nav_msgs/OccupancyGrid` which ahs an integer in the range {0, 100}.  0 is free, 100 is occupied. -1 is completely unknown

 2. Saving the map
 ```
 rosrun map_server map_saver -f name_of_the_map 
```
- They have to be saved in the `catkin_ws/src` folder
- The `yaml` file shows the origin as the LOWER LEFT POINT.

3. Providing the map for other nodes to use
- Call the `static_map` service of the `mape_server` ros service to get the map occupancy data

-  Command to launch the map server node (given the .yaml file)
```
rosrun map_server map_server map_file.yaml
```
- This publishes `/map` and `/map_metadata` topics

4. TRANSFORMS - Important
The `slam_gmapping`mnode rquires 2 transform
- **The frame attached to laser -> base_link**:mUsually a fixed vlaue, broadcast periodically by a robot_state_publisher,  or a tf static_transform_publisher.
- **Base link -> odom** Usually provided by the Odometry system

 Since the robot needs to be able toaccess this info anytime, we will publish this info to a **transform tree** . The transform tree is liek a database where we can find the info about all the transformations b/w the different frames (elements) of the robot.

```
rosrun tf view_frames
```

This allows us to visualise the transform tree as a pdf file

### Creating a launch file for the slam_gmapping node
- General parameters:
	- **base_frame(default: "base_link")**: Indicates the name of the frame attached to the mobile base
	- **map_fram (default: map)**: Indicates the name of the frame attached to the map 
	- **odom_frame (default: "odom")** : Indicates the na,e of the frame attached to the odometry system
	- **map_update_interval(default: 5.0):** 