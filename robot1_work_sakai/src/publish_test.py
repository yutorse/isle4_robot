#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def hello_talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('hello_talker')
    r = rospy.Rate(10) #10hz
    while not rospy.is_shutdown():
        str = "hello world %s"%rospy.get_time()
        rospy.loginfo(str)
        pub.publish(str)
        r.sleep()

if __name__ == '__main__':
    hello_talker()
