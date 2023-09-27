## ROS 2
1. Create a folder and then `src` inside it.
2. `cd` into this folder and type `colcon build`

### 1. Building Packges
1. `cd` into `src` folder and 
```
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake <your_package_name> --dependencies <your_package_dependencies>
```
2.  `rclcpp` is the ROS 2 C++ client library
Example 
```
ros2 pkg create --build-type ament_cmake my_robot_control --dependencies rclcpp std_msgs
```
3. Now build your workspace using `colcon build` in the `workspace` folder

### 2. Source Workspace
1. We have to source the install/setup.bash file
```
source ~/ros2_ws/install/setup.bash
```

## 3. Removing Package
1. Remove the directory.
2. `colcon build`
3. source the `setup.bash` file

## 4. Adding dependencies
1. Edit the `CMakeList.txt` 
	- 