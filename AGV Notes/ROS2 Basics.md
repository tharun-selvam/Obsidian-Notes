# ROS2
1. Create a folder and then `src` inside it.
2. `cd` into this folder and type `colcon build`
3. Resources:
	- [YT](https://www.youtube.com/watch?v=Gg25GfA456o)
## Build type *ament_cmake*
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
1. Update the `CMakeLists.txt`:
	- Add the following lines
 ```
 find_package(<package-name> REQURIRED)
```
2. Update the `package.xml`:
	- Add the following line
 ```
 <depend>PACKAGE-NAME</depend>
```
3. Run the `rosdep` command
```
rosdep install -i --from-path src --rosdistro foxy -y
```

## Build type *ament_python*
### 1. Building Packges
1. `cd` into `src` folder and 
```
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python <your_package_name> --dependencies <your_package_dependencies>
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

### 3.Writing nodes
1. `cd` into `package-name/package-name` (the created package has another folder within it of the same name)
2. Create an executable python file 
```
touch first_node.py
chmod +x first_node.py
```
3. Write the node code
4. Run it using:
	- `./first_node.py` (since the python file is executable)
5. Actual way:
	- Modify the following in `setup.py` file under `package` in `src`
 ```python
entry_points={
        'console_scripts': [
            "<executable-name> = <package-name>.<python-file-name>:main"
        ],
    }
```
- `colcon build` followed by sourcing the file `install/setup.bash` 
- `ros2 run <package-name> <executable-name>`  now runs the node

### 4. Adding dependencies
1. Just update the `package.xml` file using a `<depend>PACKAGE-NAME<depend>` tag.

### 5. Writing the code for node
**Basic Code**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node):
	def __init__(self):
		super().__init__("first_node")
		self.get_logger().info("Node has been started")
		# code to be written


def main(args=None):
	rclpy.init_node(args=args)

	# code to be written

	rclpy.shutdown()




if __name__ == '__main__':
	main() #executes the main function 





```

# Important Stuff
- Every package needs to have a `resource` folder with a file in it named `<package-name>` without any extension. This is created by default if we use the `ros2 create pkg` function.

- All the scripts must be in the folder that has the same name as the package. Alongside the `__init__.py` file.


