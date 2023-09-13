# ROS CPP 

**Resource** : [Link](https://www.youtube.com/watch?v=LkYrIQdQ6cI), ChatGPT

1. Creating a node in cpp:
	
	```cpp
		 #include <ros/ros.h> #include <std_msgs/String.h>
	```
	
	- These lines include necessary ROS headers. `ros/ros.h` provides access to ROS functionality, and `std_msgs/String.h` is the header for the `std_msgs::String` message type, which we'll use for publishing messages.
	
	
	```cpp
	int main(int argc, char** argv) {   // Initialize the ROS node   ros::init(argc, argv, "my_node");
	 ```
	
	- `int main(int argc, char** argv)` is the entry point of the C++ program. It's where execution begins. It takes command-line arguments `argc` (argument count) and `argv` (argument vector).
	- `ros::init(argc, argv, "my_node");` initializes the ROS node with the provided node name "my_node." This function should be called before creating any ROS objects. It sets up the ROS environment, initializes node-specific data, and parses ROS command-line arguments.
	
	```cpp
	// Create a ROS NodeHandle   
	ros::NodeHandle nh;
	```

	- `ros::NodeHandle nh;` creates a NodeHandle object `nh`. The NodeHandle is your interface to the ROS system, allowing you to interact with various ROS components and services. You'll use `nh` to create publishers, subscribers, and access other ROS features.
	
	
	```cpp
	   // Create a publisher that publishes a std_msgs::String message to the "my_topic" topic   
	ros::Publisher pub = nh.advertise<std_msgs::String>("my_topic", 10);
	```
	
	- `ros::Publisher pub = nh.advertise<std_msgs::String>("my_topic", 10);` creates a publisher named `pub` that publishes messages of type `std_msgs::String` to the "my_topic" topic.
	- The second argument, `10`, specifies the size of the publisher's message queue. It determines how many messages can be queued for publishing before older messages are dropped.
	
	  ```cpp
   // Set the publishing rate (e.g., 1 Hz)   
	ros::Rate rate(1);
		```
	
	- `ros::Rate rate(1);` creates a Rate object `rate` that controls the publishing rate of the node. In this example, we set the rate to 1 Hz, which means the node will attempt to publish a message every 1 second.
	
	  ```cpp
   while (ros::ok())   {     
		// Create a message     
		std_msgs::String msg;     
		msg.data = "Hello, ROS!";

		```
	
	- `while (ros::ok())` creates a loop that runs as long as the ROS node is in a running state. The condition `ros::ok()` checks if the node is still running and hasn't received a shutdown signal.
	- Inside the loop, we create a `std_msgs::String` message named `msg` and set its `data` field to "Hello, ROS!" This message contains the string we want to publish.
	
	    ```cpp
		 // Publish the message     
		 pub.publish(msg);
		```
	
	- `pub.publish(msg);` publishes the `msg` to the "my_topic" topic using the publisher `pub`.
	
	
		```cpp
		 // Sleep to maintain the specified publishing rate     
		rate.sleep();   
		}
		```
	
	- `rate.sleep();` causes the node to sleep for a duration that maintains the specified publishing rate (1 Hz in this case). It ensures that messages are published at a consistent rate.

	In summary, this C++ code defines a simple ROS node that initializes a ROS node, creates a publisher, enters a loop to publish "Hello, ROS!" messages at a specified rate, and gracefully exits when the node is signaled to shut down (`ros::ok()` returns `false`). 

 2. Running the node:
* Go into the `CMakeLists.txt` file and add the following 
	```
	 add_executable(my_node src/my_node.cpp)
	 target_link_libraries(my_node ${catkin_LIBRARIES})
	```