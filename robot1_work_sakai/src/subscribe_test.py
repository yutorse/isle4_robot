#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(message):
    rospy.loginfo("I heard %s", message.data)

def hello_listener():
    rospy.init_node('hello_listener')

    rospy.Subscriber("chatter", String, callback)

    rospy.spin()

if __name__ == '__main__':
    hello_listener()
