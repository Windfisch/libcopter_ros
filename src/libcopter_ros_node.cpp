#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
	std::cout << "msg: " << msg->linear.x << ", " << msg->linear.y << ", " << msg->linear.z << "; " << msg->angular.z << std::endl;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "libcopter_node");
	ros::NodeHandle n;

	ros::Publisher image = n.advertise<sensor_msgs::Image>("camera_raw", 5);
	ros::Subscriber cmd_vel_subscriber = n.subscribe("cmd_vel",5, cmd_vel_callback);

	ros::Rate loop_rate(10);

	cv_bridge::CvImage cv_img;

	cv_img.encoding = "bgr8";
	//cv_img.header = 
	
	int i = 0;
	while (ros::ok())
	{
		i++;
		cv_img.image = cv::Mat(100,100, CV_8UC3, cv::Scalar(i%256,255-(i%256),(i/50) % 256));

		image.publish(cv_img.toImageMsg());

		ros::spinOnce();
		loop_rate.sleep();
	}
}
