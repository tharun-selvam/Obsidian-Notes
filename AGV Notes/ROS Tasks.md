
## Task 1
* We can create a topic of any name with a message in it (as seen in code snippet 1)
* We can also write custom messages [Link](https://www.youtube.com/watch?v=baAE0i8Rzvg)

1. Created publisher
```python
#!/usr/bin/env/ python3 //setups the environment for python
import rospy
from std_msgs.msg import String

if __name__ == '__main__':
    rospy.init_node("task1_publisher")

    pub = rospy.Publisher("/my_topic", String, queue_size=10)

    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        msg = String()
        msg.data = "Hello"
        pub.publish(msg)
        rate.sleep()
```

2. Created subscriber
```python
#!/usr/bin/env/ python3 //setups the environment for python
import rospy
from std_msgs.msg import String

def string_callback(msg: String):
    rospy.loginfo(msg)

if __name__ == '__main__':
    rospy.init_node("task1_subscriber")

    sub = rospy.Subscriber("/my_topic", String, callback=string_callback)

    rospy.loginfo("Task1 Subscriber node started")
    rospy.spin()

```

## Task 2
1.  Successfully installed turtlebot3 [Link](https://www.youtube.com/watch?v=9WLBH7mNMcw)
* To launch the world in gazebo we do this:
	```
	$ cd catkin_ws/
	$ export TURTLEBOT3_MODEL=burger
	$ roslaunch turtlebot3_gazebo turtlebot3_house.launch
	```
 * To control it, we do this in another terminal:
	```
	$ cd catkin_ws/
	$ export TURTLEBOT3_MODEL=burger
	$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
	```

 2. ROS Bags
 ```
 rosbag record -a -o my_bagfile.bag
```
- **Important**: The file must end with `.bag`

3. Extract info out of ROS Bags
- `rosbag play my_bagfile.bag /cmd_vel `  will publish the topic `/cmd_vel` which will be subscribed by the node `ros_bag_subscriber.py`

4. Adding Gaussian Noise

```python
#!/usr/bin/env/ python3 //setups the environment for python
import rospy
from std_msgs.msg import Twist

def twist_callback(msg: Twist):
	mean = 0
	std_dev = .01 
	noise = np.random.normal(mean, std_dev, 3)
	
	new_msg = Twist()
	new_msg.x = msg.x + noise[0]
	new_msg.y = msg.y + noise[1]
	new_msg.z = msg.z + noise[2]

	pub.Publish(new_msg)

if __name__ == '__main__':
    rospy.init_node("ros_bag_subscriber")

    sub = rospy.Subscriber("/cmd_vel", Twist, callback=twist_callback)
	pub = rospy.Publisher("/my_topic", Twist, queue_size = 10)

    rospy.loginfo("Gaussian Node started")
    rospy.spin()

```
