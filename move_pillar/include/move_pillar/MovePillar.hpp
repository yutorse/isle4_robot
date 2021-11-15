#pragma once
// ROS
#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"

namespace move_pillar {


class MovePillar
{
 public:
  
  MovePillar(ros::NodeHandle& nodeHandle);

  virtual ~MovePillar();

 private:

  void clickCallback(const geometry_msgs::PoseStamped& msg);

  ros::NodeHandle& nodeHandle_;

  ros::Subscriber subscriber_;

  ros::Publisher pub_;

};

} 
