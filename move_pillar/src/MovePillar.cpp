#include "move_pillar/MovePillar.hpp"
#include <gazebo_msgs/ModelState.h>
#include <ros/ros.h>
#include <string>


namespace move_pillar {

  MovePillar::MovePillar(ros::NodeHandle& nodeHandle)
      : nodeHandle_(nodeHandle)
  {
    subscriber_ = nodeHandle_.subscribe("/move_base_simple/goal", 1,
                                        &MovePillar::clickCallback, this);
    
	pub_ = nodeHandle_.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10);
	
	ROS_INFO("Successfully launched node.");  

  }

  MovePillar::~MovePillar()
  {
  }

  void MovePillar::clickCallback(const geometry_msgs::PoseStamped& msg) {
   
    float pillar_x = msg.pose.position.x;
    float pillar_y = msg.pose.position.y;
    
	gazebo_msgs::ModelState new_pos;
	new_pos.model_name = "unit_cylinder";
	new_pos.pose.position.x = pillar_x;
	new_pos.pose.position.y = pillar_y;
	new_pos.pose.position.z = 1;
	pub_.publish(new_pos);
    

    ROS_INFO_STREAM("Pillar position: X value: " + std::to_string(pillar_x) + "Y value: " + std::to_string(pillar_y));

  }
}
