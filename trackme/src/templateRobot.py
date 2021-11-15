#!/usr/bin/env python

import rospy

import cv2
import tf
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped, Twist

import tf2_geometry_msgs 
import tf2_ros

def imageCallback(imgMessage):
  global pub_detected_pose, cvbridge, tfBuffer
  
  try:
    cv_image = cvbridge.imgmsg_to_cv2(imgMessage, "bgr8")
  except CvBridgeError as e:
    rospy.logerr(e)
    
  # ---------
  # Task 4-(2)
  # add your code for image processing here



  # temporary values, you should calculate them from the image
  ball_distance = 1.0
  ball_angle = 0.0
  
  # ---------
  
  
  ball_pose = PointStamped()
  
  # ball pose in the camera frame
  ball_pose.header.frame_id = imgMessage.header.frame_id
  ball_pose.header.stamp = rospy.Time()
  ball_pose.point.z = ball_distance * math.cos(ball_angle)
  ball_pose.point.x = -ball_distance * math.sin(ball_angle)

  # converting ball pose to odom frame
  try:
    trans_to_odom = tfBuffer.lookup_transform("odom", ball_pose.header.frame_id, rospy.Time())
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    rospy.logerr("Error in transforming to global frame")
    return
  ball_pose_odom = tf2_geometry_msgs.do_transform_point(ball_pose, trans_to_odom)
    
  # we do not care about the ball height (z) so we will reset it to 0
  ball_pose_odom.point.z = 0.0

  # ----------
  # Task 4-(4)
  # add temporal filtering to estimated pose
  
  
  
  # ----------
  
  pub_detected_pose.publish(ball_pose_odom)
  
  # plot image
  cv2.imshow("Original image", cv_image)
  cv2.waitKey(3)
  
	
def main():
  global pub_detected_pose, cvbridge, tfBuffer
  
  rospy.init_node('trackme_template')

  # tf2 buffer/listener
  tfBuffer = tf2_ros.Buffer()
  tfListener = tf2_ros.TransformListener(tfBuffer)
  
  # CvBridge for converting Ros <-> OpenCV
  cvbridge = CvBridge()

  # subscribe to robot image
  sub = rospy.Subscriber("camera/image", Image, imageCallback)
  
  # publisher for ball pose
  pub_detected_pose = rospy.Publisher('ball_pose', PointStamped, queue_size=10)
  
  # publisher for robot velocity command
  pub_movement = rospy.Publisher('cmd_vel', Twist, queue_size=10)
  

  rate = rospy.Rate(20.0)

  while not rospy.is_shutdown():
    
    movement = Twist()
    
    # ----------
    # Task 4-(3)
    # add code for robot control



    movement.angular.z = 0.0; # change this 
    # ----------

    movement.linear.x = 0.0; # leave this value at 0!!
    pub_movement.publish(movement);

    rate.sleep()

  cv2.destroyAllWindows()

if __name__ == '__main__':
  main()


