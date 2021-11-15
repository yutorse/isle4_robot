#!/usr/bin/env python
#!/usr/bin/env python
import rospy
import math
import cv2
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PointStamped, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

isBallDetected = False

'''self_odom_info
dst_x, dst_y = 0.0, 0.0
ball_x, ball_y = 0.0, 0.0
ball_dist = 0.0
self_x, self_y, self_z = 0.0, 0.0, 0.0
self_quaternion = [0.0, 0.0, 0.0, 0.0]

def callback_self(message):
    global self_x, self_y, self_z, self_quaternion, self_frame
    self_x = message.pose.pose.position.x
    self_y = message.pose.pose.position.y
    self_z = message.pose.pose.position.z
    self_quaternion = [message.pose.pose.orientation.x, message.pose.pose.orientation.y, message.pose.pose.orientation.z, message.pose.pose.orientation.w]

'''

Kp = 1.0

def callbackImage(msg):
  global pub_image, bridge, isBallDetected
    
  try:
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
  except CvBridgeError as e:
    rospy.logerr(e)

  low = (0, 0, 100)
  high = (30, 30, 255)
  cv_image = cv2.inRange(cv_image, low, high)

  if((cv2.findNonZero(cv_image)) is None): 
    isBallDetected = False
    rotate()
  else:
    isBallDetected = True
    move_to_ball(cv_image)


  # plot image
  
  cv2.imshow("Processed image", cv_image)
  cv2.waitKey(3)

  # convert to ROS format and publish
  try:
    pub_image.publish(bridge.cv2_to_imgmsg(cv_image, "8UC1"))
  except CvBridgeError as e:
    rospy.logerr(e)

def move_to_ball(img):
  global pub_move
  cmd_vel = Twist()
  u_0 = 160.5
  f = 265.23
      
  center = cv2.moments(img, True) # center of the ball
  u = center["m10"]/center["m00"] # x coordinate in the camera image
  #v = center["m01"]/center["m00"] # y coordinate in the camera image
  #rospy.loginfo("u=%s v=%s", u, "v")
    
  theta = math.atan2((u_0-u)/f, 1.0)
  rospy.loginfo(theta)
  rospy.loginfo(u - u_0)

  #rospy.loginfo("dst_x=%s self_x=%s beta=%s theta=%s alpha=%s", dst_x, self_x, beta, theta, alpha)
  #rospy.loginfo("dst_x=%s self_x=%s dst_y=%s self_y=%s dst_z=%s", dst_x, self_x, dst_y, self_y, dst_z)

  cmd_vel.linear.x = 0.25
  if(theta < 0):
        cmd_vel.angular.z = max(-0.5, Kp * theta)
  else:
        cmd_vel.angular.z = min(0.5, Kp * theta)
        
  pub_move.publish(cmd_vel)
      
def rotate():
  global pub_move
  cmd_vel = Twist() 
  cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.linear.z = 0.0, 0.0, 0.0
  cmd_vel.angular.x, cmd_vel.angular.y, cmd_vel.angular.z = 0.0, 0.0, 0.5
  pub_move.publish(cmd_vel)

def main():
  global pub_move, pub_image, bridge

  rospy.init_node('pickup_ball')

  # subscribe to original image
  sub_image = rospy.Subscriber("camera/image", Image, callbackImage)
  # publisher for processed image
  pub_image = rospy.Publisher("processed/image", Image, queue_size=5);

  #sub_odom = rospy.Subscriber("odom", Odometry, callback_self)

  pub_move = rospy.Publisher('cmd_vel', Twist, queue_size=10)


  # CvBridge for converting Ros <-> OpenCV
  bridge = CvBridge()

  rospy.spin()
    
if __name__ == '__main__':
    main()


