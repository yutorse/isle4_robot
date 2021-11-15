#include <ros/ros.h>
#include "move_pillar/MovePillar.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_pillar");
  ros::NodeHandle nodeHandle("~");

  move_pillar::MovePillar movePillar(nodeHandle);

  ros::spin();
  return 0;
}
