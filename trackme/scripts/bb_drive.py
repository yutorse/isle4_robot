#!/usr/bin/env python
import rospy
import numpy as np

from nav_msgs.msg import Odometry

from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
from std_srvs.srv import Empty

from visualization_msgs.msg import Marker

import tf


def eucliDist(x1,y1,x2,y2):   
	# np.abs(pose.position.x - ball["x"]) + np.abs(pose.position.y - ball["y"])
	return np.sqrt((x2-x1)**2+(y2-y1)**2)

def angleToFrom(x1,y1,x2,y2): 
	return np.arctan2(y1-y2,x1-x2)

## Process robot odometry

def odometryCb(odom):
	global robotPos
	global vx, vy
	global velPublisher
	global moveRadius

	robotPos = odom.pose.pose.position

	# marker for RViz		
	robotMarker = Marker()
	robotMarker.header = odom.header
	robotMarker.pose = odom.pose.pose
	robotMarker.type = 2 # sphere
	robotMarker.color.r = 1.0
	robotMarker.color.g = 0.0
	robotMarker.color.b = 0.0
	robotMarker.color.a = 1.0
	robotMarker.scale.x = 0.15
	robotMarker.scale.y = 0.15
	robotMarker.scale.z = 0.15

	global markerPub
	markerPub.publish(robotMarker)


	msg = Twist()

	moveRadiusX = 0.4

	# distance and angle to origin
	dist_o = np.sqrt((robotPos.x-moveCenterX)*(robotPos.x-moveCenterX)*moveRadius*moveRadius/moveRadiusX/moveRadiusX+robotPos.y*robotPos.y)
	ang_o = np.arctan2(-robotPos.y,(moveCenterX-robotPos.x)*moveRadius/moveRadiusX)

	# force towards origin
	back_force = (dist_o - moveRadius)*0.2
	if back_force < 0: back_force = 0

	vx += np.random.randn()*0.1 + back_force * np.cos(ang_o)
	vy += np.random.randn()*0.1 + back_force * np.sin(ang_o)

	vmax = 1.0

	if vx >  vmax: vx =  vmax
	if vx < -vmax: vx = -vmax
	if vy >  vmax: vy =  vmax
	if vy < -vmax: vy = -vmax


	rospy.loginfo("pos: %.1f %.1f, dist: %.1f, ang: %.1f, v: %.1f %.1f", robotPos.x, robotPos.y, dist_o, ang_o, vx, vy)

	msg.linear.x = vx
	msg.linear.y = vy
	msg.angular.z = 0
	velPublisher.publish(msg)


if __name__ == "__main__":
	rospy.init_node('bb_9')
	
	vx = 0
	vy = 0
	
	# set seed to make experiment repeatable
	# np.random.seed(10)
	
	moveRadius = 0.4
	moveCenterX = 3
	
	velPublisher = rospy.Publisher('/bb/cmd_vel', Twist, queue_size=2, latch=True)
	
	markerPub = rospy.Publisher('ballMarker', Marker, queue_size=10)
	
	rospy.Subscriber('/bb/odom',Odometry,odometryCb)
	
	rate = rospy.Rate(0.5) # svakih 1 s
	
	while not rospy.is_shutdown():
		moveRadius += 0.05
		#moveCenterX -= 0.05
		
		rospy.loginfo("moveRadius: %.1f, moveCenterX: %.1f", moveRadius, moveCenterX)
		rate.sleep()
	
	rospy.spin()
