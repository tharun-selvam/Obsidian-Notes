# ROS Exercises
## Exercise 1 [Link](https://ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/ROS2021/lec1/Exercise%20Session%201.pdf)
1. Installation 
	* Made the following:
		 1. `git` folder in `home`
		 2. `Workspaces/smb_ws/src` in `home` and `catkin_make` inside `smb_ws` 
		 3.  Created symlink of `home/tharun/git/smb_common` with the command 
		   `ln -s ~/git/smb_common <link-name>` 
		 4. `source devel/setup.bash` inside `smb_ws`
	* Launching:
		1. The setup file must be sourced first inside `smb_ws` 
		2. `cd smb_ws/src/link/smb_gazebo`  and type `roslaunch smb_gazebo smb_gazebo.launch`
	* Commanding the vehicle:
		 1. The command can be sent to the the bot by publishing the topic `/cmd_vel`
		   2. We can do that by `rostopic pub /cmd_vel <press-double-TAB>`
		 3. Pressing the double TAB would autocomplete the parameters

  1. Part 2 
	  * Installation [Reference](https://wiki.ros.org/teleop_twist_keyboard):
			1. `git clone https://github.com/ros-teleop/teleop_twist_keyboard.git` into the `~/git` repository.
			2. Created symlink into `Workspaces/smb_ws/src` using the command `ln -s ~/git/teleop_twist_keyboard`  
			3. `catkin_make` in `smb_ws` folder after sourcing `setup.bash` file.
	   * Running:
			1.  Source the setup file
			2. Go into the teleop_link (symlink)
			3. Run the command `rosrun teleop_twist_keyboard teleop_twist_keyboard.py`
			4. For this, the `smb_gazebo` must have been running
	   * Launch file:
		```xml
			<launch>
				<include file="~Workspaces/smb_ws/src/smblink/smb_gazebo/launch/smb_gazebo.launch">
					<arg name="world_file" value="($find smb_gazebo)/worlds/big_map_summer_school.world" />
				</include>
		
			</launch>
		```

 1. Error encountered
	 * When running launch file, the error `~Workspaces/smb_ws/src/smblink/smb_gazebo/launch/smb_gazebo.launch not found` came. Fixed by the following code:

	  ```xml
				<launch>
					<include file="$(find smb_gazebo)/launch/smb_gazebo.launch">
						<arg name="world_file" value="($find smb_gazebo)/worlds/big_map_summer_school.world" />
					</include>
			
				</launch>
		```
  
## Exercise 2 [Link](https://ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/ROS2021/lec2/Exercise%20Session%202.pdf)

1.  

