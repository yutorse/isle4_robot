#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry

x, y = 0.0, 0.0

def callback(message):
    x = message.pose.pose.position.x
    y = message.pose.pose.position.y
    rospy.loginfo("locaiton: x=%s y=%s", x, y)

def location():
    rospy.init_node('odometry')

    rospy.Subscriber("/odom", Odometry, callback)

    rospy.spin()

if __name__ == '__main__':
    location()
