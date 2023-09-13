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