#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def hello2():
    # initialization of ROS
    rospy.init_node('hello_ros')

    rate = rospy.Rate(10) # <= 10hz

    while not rospy.is_shutdown():

        hello_str = "Hello, ROS! %s" % rospy.get_time
        rospy.loginfo(hello_str)

        rate.sleep() # <=

# execute the hello2 function
hello2()