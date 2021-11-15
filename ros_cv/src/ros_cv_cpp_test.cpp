#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

image_transport::Publisher pub;

void callbackImage(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
          
    cv::imshow("Picture", cv_ptr->image);
    cv::waitKey(3);

    pub.publish(cv_ptr->toImageMsg());
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");
    ros::NodeHandle nh_;
    
    cv::namedWindow("image");
    
    image_transport::ImageTransport it_(nh_);
    image_transport::Subscriber pret = it_.subscribe("/myimage", 1, &callbackImage);
    pub = it_.advertise("/processed/image", 1);

    ros::spin();    
    return 0;
}
