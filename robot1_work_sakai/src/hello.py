#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def hello():
    # initialization of ROS
    rospy.init_node('hello_ros')

    # show the log
    rospy.loginfo("Hello, ROS!")

# execute the hello function
hello()