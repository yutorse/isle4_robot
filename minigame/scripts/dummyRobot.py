#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


def cbOdometry(odomMessage):
  global pub_speed
  speed = Twist() 
  speed.linear.x = 0.0
  speed.angular.z = 0.0
  pub_speed.publish(speed)

	
def main():
  global pub_speed
  rospy.init_node('solution2')

  rospy.Subscriber("odom", Odometry, cbOdometry)
  pub_speed = rospy.Publisher('cmd_vel', Twist, queue_size=10)
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
    

if __name__ == '__main__':
  main()

