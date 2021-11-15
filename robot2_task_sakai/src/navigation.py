#!/usr/bin/env python
import rospy
import tf
import math
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


cmd_vel = Twist()
cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.linear.z = 0.0, 0.0, 0.0
cmd_vel.angular.x, cmd_vel.angular.y, cmd_vel.angular.z = 0.0, 0.0, 1.0

dst_x, dst_y, dst_z = 0.0, 0.0, 0.0
self_x, self_y, self_z = 0.0, 0.0, 0.0
self_quaternion = [0.0, 0.0, 0.0, 0.0]

Kp = 0.5

def callback_dst(message):
    global dst_x, dst_y, dst_z
    dst_x = message.pose.position.x
    dst_y = message.pose.position.y
    dst_z = message.pose.position.z

def callback_self(message):
    global self_x, self_y, self_z, self_quaternion
    self_x = message.pose.pose.position.x
    self_y = message.pose.pose.position.y
    self_z = message.pose.pose.position.z
    self_quaternion = [message.pose.pose.orientation.x, message.pose.pose.orientation.y, message.pose.pose.orientation.z, message.pose.pose.orientation.w]

def navigation():
    rospy.init_node('navigation')

    sub_dst = rospy.Subscriber("/move_base_simple/goal", PoseStamped, callback_dst)
    sub_self = rospy.Subscriber("/odom", Odometry, callback_self)
    
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    r = rospy.Rate(20)

    while not rospy.is_shutdown():
        beta = math.atan2((dst_y - self_y), (dst_x - self_x))
        theta = tf.transformations.euler_from_quaternion(self_quaternion)[2]
        alpha = beta - theta

        rospy.loginfo("dst_x=%s self_x=%s beta=%s theta=%s alpha=%s", dst_x, self_x, beta, theta, alpha)

        if ((abs(dst_x - self_x)) ** 2 + (abs(dst_y - self_y)) ** 2 < 0.005):
            cmd_vel.linear.x = 0.0
            cmd_vel.linear.y = 0.0
            cmd_vel.linear.z = 0.0
            cmd_vel.angular.x = 0.0
            cmd_vel.angular.y = 0.0
            cmd_vel.angular.z = 0.0
        else:
            if(abs(alpha) > math.pi):
                cmd_vel.linear.x = 0.0
            else:
                cmd_vel.linear.x = 0.1

        cmd_vel.angular.z = min(0.5, Kp * alpha)        
        pub.publish(cmd_vel)
        r.sleep()

if __name__ == '__main__':
    navigation()