# ROS Basics

* Typing `roscore` activates the ros master. Can only be done once simultaneously. It is done in the home directory.
* `rqt_graph` : Shows a visual represnetation of nodes, topics and communications.
* **Important**: `code --disable-gpu` command should be run to launch vs code directly

## Installation
* See the video linked below.
*  We install three extensions for VS Code 
	1. Python
	2. cmake from txws
	 3. ROS from Microsoft
* Everytime roscore is run, we have to source the following command in the home directory to setup an environment:
	```
	 $ source /opt/ros/noetic/setup.bash
	```
* We have to run this source command mentioned above everytime we run roscore in a new terminal. So, instead we type the following once:
	```
	 $ echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc 
	```
*  `geddit ~/.bashrc` to check the command is at the last.

## Turtlesim
1. `rosrun turtlesim turtlesim_node` starts the GUI of turtlesim
2. `rosrun turtlesim draw_square` draws a square 
3. `rosrun turtlesime turtle_teleop_key` lets us control the turtle using keys


## Node
* Any program you could create with code is a node. They can communicate between eachother.
* Already ros master comes with some nodes we can use.
* Nodes are organised in packages.
* Running the same node again when it is already running kills the execution of the first started node and starts the execution of the called node again.
* Running a node
	* `rosrun <package-name> <node-name>` : Runs the node.
	* `rosnode list ` lists all the activley running nodes. `/rosout` is started by default when `roscore` command is given.
	* Example: 
 ```
 $ rosrun rospy_tutorials talker
```
* Shutting down a node:
	1. `Cmd + C` to stop the execution.
	2.  `kill <node-name>` shuts down the particular node

## Catkin Workspace
1. First Time Setup
	* We create a catkin workspace to crate our own nodes and packages.
	* We first need to create a folder. `mkdir <folder-name>`. Ex : `mkdir catkin_ws`.
	* `cd catkin_ws` and create a folder named `src`, using the command `mkdir src`.
	* Now type `catkin_make` inside the `catkin_ws` folder to setup the other stuff.
	* The`catkin_ws` now has `build devel src` folders.
	* Go to the home directory and type 
	```
	$ source ~/catkin_ws/devel/setup.bash 
	$ gedit ~/.bashrc
	// now paste the above command into the window popup
	```
2. Compiling the workspace
* Write `catkin_make` inside the workspace directory to compile new packages and nodes as we write them.
* We needn't however compile every time we make a new node if the node is written in python. Not the case for cpp.
## ROS Package
* We always write nodes inside packages.
* cd into the src folder. 
* We also have to mention the libraries that the package is going to need so that the nodes inside this package can talk to nodes outside this package.
* `catkin_create_pkg <package-name> <library-dependencies-with-space>`
* Ex: `catkin_create_pkg my_robot_controller rospy roscpp turtlesim`
* `code .` command inside the src folder opens VS Code
* To add more dependencies later on, we have to go into the `.xml` file in the `src` folder of the package and do :
	```xml
	  <build_depend>package-name</build_depend>
	  <build_export_depend>package-name</build_export_depend>
	  <exec_depend>package-name</exec_depend>
	 ```

## Writing a node
1. Creating the python file
	```python
	$ cd catkin_ws/src/my_robot_controller/src
	$ mkdir scripts
	$ cd scripts/
	$ touch my_first_node.py
	$ chmod +x my_first_node.py //makes the py file executable
	```
1. Inside the python file
	```python
	#!/usr/bin/env/ python3 //setups the environment for python
	import rospy
	
	if __name__ == '__main__':
		rospy.init_node("test_node") #creates a node called test_node
	
		rospy.loginfo("Hello from test node")
		rospy.logwarn("This is a warning")
		rospy.logerr("This is an error")
	
		rospy.sleep(1.0)
	
		rospy.loginfo("End of program")
	
	```

 1. Writing a code to print a message continuously
 ```python
 #!/usr/bin/env/ python3 //setups the environment for python
import rospy

if __name__ == '__main__':
	rospy.init_node("test_node") #creates a node called test_node

	rospy.loginfo("Test node started")

	rate = rospy.Rate(10) # specifies the frequency in Hz 

	while not rospy.is_shutdown():
		 rospy.loginfo("Hello")
		 rate.sleep().   # prints the above message in the frequency specified above
	

 ```
 
 1. Running the node
	 * First run the command `roscore` in the home directory
	 * `python3 my_first_node.py` to be run in scripts folder inside the package.
	 *  `./my_first_node.py` can also be used
	 * `rosrun my_robot_controller my_first_node.py` on the home directory also runs the node.
  
## Topic
* ROS Topics are how nodes communicate with eachother. If a node wants to send a message, it publishes a topic to which other nodes can subscribe.
* In this case a subscriber will now have access to the contents of the topic.
* Example, `/chatter` is a topic name
*  `rostopic list` lists all the topics currently being used.
	* `rostopic info /chatter` lists info about the topic. It specifies Publishers, Listeners and the message type.
	* `rosmsg show std_msgs/String (the type of data carried by the topic)` shoes the type of data carried by the topic. 

	* `rostopic echo /chatter` lets us see what the topic contains. 
	* `rostopic hz /chatter` specifies the frequency of the topic
 * It is represented by a square box in the rqt graph.
 * We can also create custom topics by simply specifying it's name. (Refer [ROS Task 1](obsidian://open?vault=Obsidian&file=AGV%20Notes%2FROS%20Task%201))
	### Custom Messages
	1. We can create custom messages [Link](https://www.youtube.com/watch?v=baAE0i8Rzvg)

## Publisher
1. Create a node as above and do the following code

```python
#!/usr/bin/env/ python3 //setups the environment for python
import rospy
form geometry_msgs.msg import Twist #type of mssage this node publishes 

if __name__ == '__main__':
	rospy.init_node("draw_circle")
	
	pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10) # pub is a Publisher() object with attributes topic name, topic msg, queue_size is a buffer which holds 10 mssgs
	
	#send a topic every 0.5 seconds
	
	rate = rospy.Rate(2)
	
	while not rospy.is_shutdown():
		#create msg
		msg = Twist()
		msg.linear.x = 2.0
		msg.angular.z = 1.0
		
		#send the message
		pub.publish(msg)
		
		#sets the frequency to 2Hz
		rate.sleep()  
```

## Subscriber
* We are creating a subscriber that subscribes to `/turtle1/pose` which is published by `turtlesim_node` 
* The following file is named `pose_subscriber.py`
1. Create a node as above and do the following code
 ```python
#!/usr/bin/env/ python3 //setups the environment for python
import rospy
form turtlesim.msg import Pose # Pose is the type of mssage this node is going to subsribe to 

def pose_callback(msg: Pose): # msg: Pose specifies that the argument will be a Pose
	rospy.loginfo(msg)

if __name__ == '__main__':
	rospy.init_node("turtle_pose_subscriber")

	sub = rospy.Subscriber("/turtle1/pose", Pose, callback=pose_callback) # sub is a Subscriber() object with attributes topic name, topic msg, callback is a function which will be called when mssg is received by the listener

	rospy.loginfo("Node has been started")
	rospy.spin() #blocks until ROS is shutdown. yields activity to other threads
```

## Subscriber and Publisher in a Closed Loop
* `turtle_controller.py` - This node subscribes to `/turtle1/pose` and publish the topic `/turtle1/cmd_vel`.
* This particular script will make the turlte go to the left until it hits the wall and then make a turn and continue going.
```python
#!/usr/bin/env/ python3 
import rospy
from turtlesim.msg import Pose 
from geometry_msgs.msg import Twist

def pose_callback(pose: Pose): 
	cmd = Twist()
 
	if pose.x > 9.0 or pose.x < 2.0 or pose.y > 9.0 or pose.y < 2.0:
		cmd.linear.x = 1.0
		cmd.angular.z. = 1.4
	else:
		cmd.linear.x = 5.0
		cmd.angluar.z = 0.0
  
	cmd.linear.x = 5.0
	cmd.angular.z = 0.0
	pub.Publish(cmd)

if __name__ == '__main__':
	rospy.init_node("turtle_controller")
	pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
	sub = rospy.Subscriber("/turtle1/pose", Pose, callback=pose_callback) 
	rospy.loginfo("Node has been started")
 
	rospy.spin() 
```

## ROS Service
* `rosrun rospy_tutorials add_two_ints_server`: Starts a server that adds two ints
* Same as the last time, `rosservice list` lists the current services and `rosservice info <service-name>` gives the details of the service.
* `rosservice call /add_two_ints "a: 5 b:3"`. Press `Tab` twice to show to autocomplete arguments.
* Analogy: Web browser is the client and the web server is the server
* Computation, change the setting - Uses of ros service
* Just like `rosmsg show <msg-type-inside-a-topic>` the similar command is `rossrv show <type-inside-service>`
* `rossrv show rospy_tutorial/AddTwoInts`, in the output the type of 'output' that the service provides (after receiving the arguments) is shown after three dashes `---`
* ROS Services differ from topics in the fact that here, there is a client server relationship.
* One important thing is that services are not meant to be called at high frquencies like topics 
1. Writing a server client code:
	* We write a service in the existing `turtle_controller.py`  file. We wannt the turtle's pen colour to change if the turtle is in different halves of the screen.
 
```python
#!/usr/bin/env/ python3 
import rospy
from turtlesim.msg import Pose 
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen #SetPen is the type of "srv" (similar to "msg" of a topic) that /turtle1/set_pen uses

previous_x = 0.0

def call_set_pen_service(r, g, b, width, off):
	# responsible for calling the set_pen service
	try:
		# ServiceProxy function is responsible for creating a client
		set_pen = rospy.ServiceProxy("/turtle1/set_pen", SetPen)
		response = set_pen(r, g, b, width, off) #order is important
		# rospy.loginfo(response) (we can print the response received which in this case is nothing)
	except rospy.ServiceException as e:
		rospy.logwarn(e)
		

def pose_callback(pose: Pose): 
	cmd = Twist()
 
	if pose.x > 9.0 or pose.x < 2.0 or pose.y > 9.0 or pose.y < 2.0:
		cmd.linear.x = 1.0
		cmd.angular.z. = 1.4
	else:
		cmd.linear.x = 5.0
		cmd.angluar.z = 0.0
  
	cmd.linear.x = 5.0
	cmd.angular.z = 0.0
	pub.Publish(cmd)

	# this variable previous_x ensures that the service is only called when the turtle crosses halves
	global previous_x
	if pose.x >= 5.5 and previous_x < 5.5:
		rospy.loginfo("Set color to red!")
		call_set_pen_service(255, 0, 0, 3, 0)
	elif pose.x < 5.5 and previous_x >= 5.5:
		rospy.loginfo("Set color to red!")
		call_set_pen_service(0, 255, 0, 3, 0)
	previous_x = pose.x

if __name__ == '__main__':
	rospy.init_node("turtle_controller")
	rospy.wait_for_service("/turtle1/set_pen") #waits for the service to be up
	
	pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
	sub = rospy.Subscriber("/turtle1/pose", Pose, callback=pose_callback) 
	rospy.loginfo("Node has been started")
 
	rospy.spin() 
```
## ROS Bags
* `cd` into `catkin_ws/src/bagfiles` and then do the following
1. `rosbag record <topic-names-with-space>` : Records the specified topics into a bagfile that it creates.
2. `rosbag record -a` : Records all topics
3. `rosbag info <rosbag_filename>`
4. `rosbag record -a -O <bagfile-name>` : Records all the topics into the bagfile name specified which has to end with `.bag` 

## ROS Launch files
* ROS launch files serve several key purposes:
	- Simplify the process of starting multiple ROS nodes together.
	- Set parameters and remap topics for nodes.
	- Organize and manage the launch of nodes within ROS packages.
	- Make it easier to launch complex robot systems with various configurations.
 * Written with `.launch` extension in XML(preferred) or YAML
 * Written inside `workspace/src/package/launch` 
 1. Structure
	- `<launch>`: The root element that encapsulates the entire launch file.
	- `<node>`: This element is used to specify a ROS node to launch. You can include multiple `<node>` tags to launch multiple nodes.
	- `<param>`: Used to set parameters for nodes.
	- `<include>`: Allows you to include other launch files, enabling modularity and reusability.
	- `<arg>`: Defines arguments that can be passed to the launch file or other included launch files.
		```xml
		<launch>
		  <node name="my_node" pkg="my_package" type="my_node_type" output="screen">
			<param name="param_name" value="param_value"/>
		  </node>
		</launch>
		```
 
2. Attributes and Elements:
	- `name`: Specifies a unique name for the node. This is essential for distinguishing between nodes.
	- `pkg`: The name of the ROS package containing the node's executable.
	- `type`: The name of the node's executable within the package.
	- `output`: Determines where the node's output is directed. Common values include "screen" (output to the console) and "log" (output to log files).
	- `<param>`: Defines parameters for nodes, which can be read by the nodes during runtime.
	- `<include>`: Allows you to include other launch files to compose more complex launch configurations.
	- `<arg>`: Defines arguments that can be passed to the launch file, allowing for runtime customization.
3. Running Launch files:
	```
	roslaunch my_package my_launch_file.launch
	 ```






## Resources
* [Watched this video first](https://www.youtube.com/watch?v=wfDJAYTMTdk)
