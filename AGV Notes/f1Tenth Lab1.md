# f1TenthLab 1
1. Executed the command which runs a container containing the ros2 image
```
docker run -it -v <absolute_path_to_this_repo>/lab1_ws/src/:/lab1_ws/src/ --name f1tenth_lab1 ros:foxy
```
- It opened a new terminal inside the container. This is an interactive container so this opens up a terminal.
1. Ran `apt update && apt install tmux`. `tmux` allows us to have multiple `bash` sessions in the same terminal window
2. Typing `tmux` activates it. Now we can press `Ctrl + B` and `C` to create a new window

## Creating Package 
1. Use `ros2 pkg create --build-type ament_cmake lab1_pkg` to create the pkg
2. Update the `CMakeLists.txt`:
	- Add the following lines
 ```
 find_package(rclcpp REQURIRED)
 find_package(rclpy REQURIRED)
 find_package(ackermann_msgs REQURIRED)
```
3. Update the `package.xml`:
	- Add the followign line
 ```
 <depend>rclcpp</depend>
 <depend>rclpy</depend>
 <depend>ackermann_msgs</depend>
```
4. Run the following command
```
rosdep install -i --from-path src --rosdistro foxy -y
```

## Creating Package (Alternate)
1. Created package using `ament_python` which is much easier to work with
```
$ ros2 pkg create --build-type ament_python lab1_pkg rclpy ackermann_msgs
```
## Errors 
1. Restarting the container:  The `-i` is important
```
sudo docker container start -i f1tenth_lab1
```

2. Couldn't install `ackermann_msgs` dependency. 
	- Fixed after running the command:
```
rosdep install -i --from-path src --rosdistro foxy -y
```


