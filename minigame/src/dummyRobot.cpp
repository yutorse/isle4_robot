#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>


ros::Publisher pub_speed;

void odomHandler(const nav_msgs::Odometry& pozicija) {
	geometry_msgs::Twist speed;
	speed.linear.x = 0.0;
	speed.angular.z = 0.0;
	pub_speed.publish(speed);
	
}

int main (int argc, char **argv) {
	ros::init(argc, argv, "glupi_robot");
	ros::NodeHandle nh;

	ros::Subscriber sub_odom = nh.subscribe("odom", 10, &odomHandler);
	pub_speed = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	
	ros::spin();
	
	return 0;

	/* 
	 * primjer korištenja čarolije 
	 * 
	std_srvs::Empty srv;
	ros::Rate loop(1);
	
	while(ros::ok()) {
		
		// koristi čaroliju
		int spellNum;
		if (nh.getParam("spell", spellNum)) {
			if (spellNum > 0) {
				ROS_INFO("SPELL!!!");
				ros::service::call("/petrificus", srv);
			}
		}
		ros::spinOnce();
		loop.sleep();
	}
	*/

}
