#!/usr/bin/env python
import rospy
import tf
import tf2_ros
import math
import tf2_geometry_msgs
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PointStamped, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty, EmptyResponse

cmd_vel = Twist()

dst_x, dst_y = 0.0, 0.0
pillar_x, pillar_y = 0.0, 0.0
pillar_dist = 0.0
self_x, self_y, self_z = 0.0, 0.0, 0.0
self_quaternion = [0.0, 0.0, 0.0, 0.0]

Kp = 1.0

def callback_pillar(message):
    global pillar_x, pillar_y, pillar_dist

    ranges = message.ranges
    angle_min = message.angle_min
    angle_increment = message.angle_increment

    pillar_index = 0 #index of the closest object in ranges
    tmp_dist = float('inf')
    for index, data in enumerate(ranges):
        if(tmp_dist > data and data != 0):
            tmp_dist = data
            pillar_index = index

    pillar_dist = tmp_dist # distance for the closest object
    pillar_angle = angle_min + pillar_index * angle_increment # angle for the closest object

    pillar_x = pillar_dist * math.cos(pillar_angle)
    pillar_y = pillar_dist * math.sin(pillar_angle)
    #rospy.loginfo("range=%s angle=%s", ranges, pillar_angle)
    rospy.loginfo("pillar_dist=%s pillar_index=%s pillar_angle_y=%s", pillar_dist, pillar_index, pillar_angle)

def callback_self(message):
    global self_x, self_y, self_z, self_quaternion, self_frame
    self_x = message.pose.pose.position.x
    self_y = message.pose.pose.position.y
    self_z = message.pose.pose.position.z
    self_quaternion = [message.pose.pose.orientation.x, message.pose.pose.orientation.y, message.pose.pose.orientation.z, message.pose.pose.orientation.w]

def navigation_pillar(req):
    #rospy.loginfo("pillar_dist=%s", pillar_dist)
    sub_pillar = rospy.Subscriber('/scan', LaserScan, callback_pillar)
    sub_self = rospy.Subscriber("/odom", Odometry, callback_self)
    
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    r = rospy.Rate(20)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform("odom", "base_scan", rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            r.sleep()
            continue
        
        pillar_point = Point(pillar_x, pillar_y, 0)
        dst_x = tf2_geometry_msgs.do_transform_point(PointStamped(point = pillar_point), trans).point.x
        dst_y = tf2_geometry_msgs.do_transform_point(PointStamped(point = pillar_point), trans).point.y
        beta = math.atan2((dst_y - self_y), (dst_x - self_x))
        theta = tf.transformations.euler_from_quaternion(self_quaternion)[2]
        alpha = beta - theta

        #rospy.loginfo("dst_x=%s self_x=%s beta=%s theta=%s alpha=%s", dst_x, self_x, beta, theta, alpha)
        #rospy.loginfo("dst_x=%s self_x=%s dst_y=%s self_y=%s dst_z=%s", dst_x, self_x, dst_y, self_y, dst_z)
        #rospy.loginfo("pillar_x=%s pillar_y=%s", pillar_x, pillar_y)
        if(pillar_dist < 0.3):
            cmd_vel.linear.x = 0.0
            cmd_vel.linear.y = 0.0
            cmd_vel.linear.z = 0.0
            cmd_vel.angular.x = 0.0
            cmd_vel.angular.y = 0.0
            cmd_vel.angular.z = 0.0
            pub.publish(cmd_vel)
            break
        else:
            if(abs(alpha) > math.pi):
                cmd_vel.linear.x = 0.0
            else:
                cmd_vel.linear.x = 0.2
            rospy.loginfo("here")
            pub.publish(cmd_vel)
        cmd_vel.angular.z = min(0.5, Kp * alpha)        
        r.sleep()

    return EmptyResponse()

def goto_closest():
    rospy.init_node("goto_closest_server")
    s = rospy.Service("goto_closest", Empty, navigation_pillar)
   
    rospy.spin()
    

if __name__ == '__main__':
    goto_closest()