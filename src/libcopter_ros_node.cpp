#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include "sg500.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

static SG500 drone;

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
	drone.command(-msg->linear.y, -msg->linear.x, -msg->angular.z, msg->linear.z);
}

void cmd_takeoff_callback(const std_msgs::Empty& msg)
{
	ROS_DEBUG("takeoff");
	drone.takeoff();
}

void cmd_land_callback(const std_msgs::Empty& msg)
{
	ROS_DEBUG("land");
	drone.land();
}

void cmd_panic_callback(const std_msgs::Empty& msg)
{
	ROS_DEBUG("panic");
	drone.panic();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "libcopter_node");
	ros::NodeHandle n;

	ros::Publisher image = n.advertise<sensor_msgs::Image>("camera_raw", 5);
	ros::Subscriber cmd_vel_subscriber = n.subscribe("cmd_vel",5, cmd_vel_callback);
	ros::Subscriber takeoff_subscriber = n.subscribe("takeoff",1, cmd_takeoff_callback);
	ros::Subscriber land_subscriber = n.subscribe("land",1, cmd_land_callback);
	ros::Subscriber panic_subscriber = n.subscribe("panic",1, cmd_panic_callback);

	ros::Rate loop_rate(90);

	cv_bridge::CvImage cv_img;
	cv_img.encoding = "bgr8";

	if (!drone.initialize())
	{
		ROS_FATAL("Failed to initialize drone!");
		exit(1);
	}
	
	int framecnt = 0;
	std::optional<double> t0 = std::nullopt;
	while (ros::ok())
	{
		auto [video_frames, telemetry_frames] = drone.poll_data();
		framecnt += video_frames.size();
		if (video_frames.size() > 0)
		{
			auto& frame = video_frames.back();
			cv_img.image = frame.frame;
			cv_img.header.seq = framecnt-1;

			double t = frame.timestamp;
			if (!t0.has_value())
				t0 = t - std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 1000000.0;
			
			cv_img.header.stamp.fromSec(t - *t0);
			
			image.publish(cv_img.toImageMsg());
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
}
