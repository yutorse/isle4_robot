#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import PolygonStamped, PoseStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty

self_x, self_y, self_z = 0.0, 0.0, 0.0

ghost_pose = PoseStamped()
ghost_pose.header.frame_id = "map"
ghost_pose.pose.orientation.x = 0.0
ghost_pose.pose.orientation.y = 0.0
ghost_pose.pose.orientation.z = 0.0
ghost_pose.pose.orientation.w = 1.0

ghosts = []

def calc_dist(point):
  x, y, z = point.x, point.y, point.z
  dist = math.sqrt((self_x - x)**2 + (self_y- y)**2 + (self_z - z)**2)
  return dist

def callback_self(msg):
  global self_x, self_y, self_z
  self_x = msg.pose.pose.position.x
  self_y = msg.pose.pose.position.y
  self_z = msg.pose.pose.position.z
  #rospy.loginfo("odom = %s", self_x)

def callback_ghost(msg):
  global ghosts
  ghosts = msg.polygon.points

def move_ghost():
  global pub_goal
  if(len(ghosts) != 0):
    ghost_distances = list(map(calc_dist, ghosts))
    closest_distance = min(ghost_distances)
    closest_index = ghost_distances.index(min(ghost_distances))

    ghost_pose.header.stamp = rospy.Time.now()
    ghost_pose.pose.position.x = ghosts[closest_index].x
    ghost_pose.pose.position.y = ghosts[closest_index].y
    ghost_pose.pose.position.z = ghosts[closest_index].z

    if(closest_distance <= 0.5):
      rospy.wait_for_service('/buster')
      try:
        service = rospy.ServiceProxy('/buster', Empty)
        del ghosts[closest_index]
        service()
      except rospy.ServiceException as e:
        rospy.logerr("buster failed: %s"%e)

    pub_goal.publish(ghost_pose) 

def main():
  global pub_goal, sub_ghost, sub_odom
  rospy.init_node('ghost_buster')

  sub_odom = rospy.Subscriber("/odom", Odometry, callback_self)
  sub_ghost = rospy.Subscriber("/ghost", PolygonStamped, callback_ghost)

  pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

  rate = rospy.Rate(3.0)

  while not rospy.is_shutdown():
    move_ghost()
    rate.sleep()

if __name__ == '__main__':
  main()
