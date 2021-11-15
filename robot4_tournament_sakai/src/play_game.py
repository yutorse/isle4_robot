#!/usr/bin/env python
import rospy
import tf
import tf2_ros
import tf2_geometry_msgs
import cv2
from math import atan2, degrees, sqrt, cos, sin
from geometry_msgs.msg import Twist, Point, PointStamped, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Wrench
from std_msgs.msg import Int8
from std_srvs.srv import Empty

k = 0.05

x = 0.0
y = 0.0

goal_x = 0.0
goal_y = 0.0

angle = 0.0

dr = 0.0

pub_vel = 0
pub_marker = 0
tfBuffer = 0
tfListener = 0

low = (0, 0, 100)
high = (30, 30, 255)

def loc_callback(message):
    global x, y, angle
    x = message.pose.pose.position.x
    y = message.pose.pose.position.y
    ori = message.pose.pose.orientation
    e = tf.transformations.euler_from_quaternion((ori.x, ori.y, ori.z, ori.w))
    angle = degrees(e[2])

def find_redball(image):
    # TODO: refine range of red
    image = cv2.inRange(image, low, high)

    if cv2.findNonZero(image) is None:
        rospy.loginfo("not found")
        return 0.0, False
    
    # calc center
    mu = cv2.moments(image, True)
    x = int(mu["m10"] / mu["m00"])

    rospy.loginfo("x: %f", x)
    return x, True

def action_is_found(u):
    global goal_x, goal_y
    ry = 10.0
    u0  = 160.5
    f = 265.23
    rx = (u - u0) * ry / f

    twist = Twist()
    vel, angule_vel = calc_vel(rx, ry)
    twist.linear.x = vel
    twist.angular.z = angule_vel
    rospy.loginfo("%f, %f", vel, angule_vel)
    pub_vel.publish(twist)

def action_not_found():
    twist = Twist()
    twist.linear.x = 0
    twist.angular.z = 0.5
    pub_vel.publish(twist)

def image_callback(message):
    global bridge
    rospy.loginfo("Subscribed")

    cv_image = bridge.imgmsg_to_cv2(message, "bgr8")

    image = cv2.inRange(cv_image, low, high)
    # find red ball BGR
    find_redball(cv_image)
    
    cv2.imshow("Raw image", image)
    cv2.waitKey(3)

    x, is_found = find_redball(cv_image)

    if (is_found):
        action_is_found(x)
    else:
        action_not_found()

def calc_vel(dx, dy):
    # calc angle
    angle_diff = degrees(atan2(dy, dx)) - 90

    # map angle_diff [-360, 360] -> [-180, 180]
    if angle_diff < -180:
        angle_diff += 360
    if angle_diff > 180:
        angle_diff -= 360
    rospy.loginfo(angle_diff)

    # calc anguoar velocity
    angle_vel = min(abs(k * angle_diff), 0.5)
    if angle_diff < 0:
        angle_vel *= -1

    # calc velocity
    vel = 0.01
    if abs(angle_diff) < 45:
        vel = 0.25

    return vel, angle_vel


world_size = 7


def eucliDist(x1,y1,x2,y2):   
	# np.abs(pose.position.x - ball["x"]) + np.abs(pose.position.y - ball["y"])
	return np.sqrt((x2-x1)**2+(y2-y1)**2)

def angleToFrom(x1,y1,x2,y2): 
	return np.arctan2(y1-y2,x1-x2)

## Process robot odometry

def odometryCb0(msg):
	global robotPoses
	robotPoses[0] = msg.pose.pose

def odometryCb1(msg):
	global robotPoses
	robotPoses[1] = msg.pose.pose

def odometryCb2(msg):
	global robotPoses
	robotPoses[2] = msg.pose.pose

def odometryCb3(msg):
	global robotPoses
	robotPoses[3] = msg.pose.pose

def odometryCbMy(msg):
    global myPos
    myPos = msg.pose.pose

def isInFront(checkX, checkY):
    dist = eucliDist(checkX, checkY, myPos.position.x, myPos.position.y)
    if dist > 3:
        return False
    angle = angleToFrom(checkX, checkY, myPos.position.x, myPos.position.y)
    rQuat = [myPos.orientation.x, myPos.orientation.y, myPos.orientation.z, myPos.orientation.w]
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(rQuat)
    angle_diff = angle-yaw
    while angle_diff > np.pi: angle_diff -= 2*np.pi
    while angle_diff < -np.pi: angle_diff += 2*np.pi
    angle_diff = np.abs(angle_diff)
    if angle_diff > np.pi/6:
        return False
    if angle_diff < np.pi/10000000000 and dist < 0.0000000000001:
        return False
    return True
	
robotPoses=[]
myPos=[]

if __name__ == "__main__":
    for i in range(4):
        robotPoses.append(Pose())
    myPos = Pose()
    pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('play_game')

    rospy.loginfo("start")

    bridge = CvBridge()
    tfBuffer =tf2_ros. Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)

    rospy.Subscriber("odom", Odometry, loc_callback)
    rospy.Subscriber("camera/image", Image, image_callback)
	
    rospy.Subscriber('/robo0/odom',Odometry,odometryCb0)
    rospy.Subscriber('/robo1/odom',Odometry,odometryCb1)
    rospy.Subscriber('/robo2/odom',Odometry,odometryCb2)
    rospy.Subscriber('/robo3/odom',Odometry,odometryCb3)
	
    rospy.wait_for_service('/petrificus')
    petrificus = rospy.ServiceProxy('/petrificus', Empty)
    rospy.wait_for_service('/kamehameha')
    kamehameha = rospy.ServiceProxy('/kamehameha', Empty)

    #while not gameOver and not rospy.is_shutdown():
    #	waitrate.sleep()
    while(not rospy.is_shutdown()):
        for i in range(4):
            if isInFront(robotPoses[i].position.x, robotPoses[i].position.y):
                rospy.loginfo("petrificus!!!!!!")
                petrificus()
                break