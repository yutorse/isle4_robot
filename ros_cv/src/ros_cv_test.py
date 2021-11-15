#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def callbackImage(msg):
  global pub, bridge
    
  try:
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
  except CvBridgeError as e:
    rospy.logerr(e)

  # process image - foe example, draw a circle
  (rows,cols,channels) = cv_image.shape
  
  if cols > 60 and rows > 60 :
    cv2.circle(cv_image, (50,50), 10, 255)

  low = (0, 0, 100)
  high = (30, 30, 255)
  cv_image = cv2.inRange(cv_image, low, high)

  # plot image
  cv2.imshow("Processed image", cv_image)
  cv2.waitKey(3)

  # convert to ROS format and publish
  try:
    pub.publish(bridge.cv2_to_imgmsg(cv_image, "8UC1"))
  except CvBridgeError as e:
    rospy.logerr(e)

def main():

  global pub, bridge

  rospy.init_node('image_converter', anonymous=True)

  # subscribe to original image
  sub = rospy.Subscriber("/camera/image", Image, callbackImage)
  # publisher for processed image
  pub = rospy.Publisher("/processed/image", Image, queue_size=5);

  # CvBridge for converting Ros <-> OpenCV
  bridge = CvBridge()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
