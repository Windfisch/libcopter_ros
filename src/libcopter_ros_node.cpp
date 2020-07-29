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

/** converts a drone timestamp into ROS format */
ros::Time timestamp(double drone_timestamp /* [sec] */)
{
	static std::optional<ros::Duration> t0 = std::nullopt;
	static std::optional<double> t0_drone = std::nullopt;

	if (!t0_drone.has_value()) t0_drone = drone_timestamp;
	std::cout << drone_timestamp << " - " << *t0_drone << std::endl;
	ros::Time t_raw(drone_timestamp-*t0_drone);

	
	if (!t0.has_value()) t0 = t_raw - ros::Time::now();
	ROS_DEBUG_THROTTLE(1, "timestamp age: %5.1f ms", 1000.0*((t_raw - *t0 - ros::Time::now()).toSec()) );
	return t_raw - *t0;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "libcopter_node");
	ros::NodeHandle n;
	ros::Rate loop_rate(90);

	ros::Publisher image = n.advertise<sensor_msgs::Image>("camera_raw", 5);
	ros::Subscriber cmd_vel_subscriber = n.subscribe("cmd_vel",5, cmd_vel_callback);
	ros::Subscriber takeoff_subscriber = n.subscribe("takeoff",1, cmd_takeoff_callback);
	ros::Subscriber land_subscriber = n.subscribe("land",1, cmd_land_callback);
	ros::Subscriber panic_subscriber = n.subscribe("panic",1, cmd_panic_callback);

	cv_bridge::CvImage cv_img;
	cv_img.encoding = "bgr8";

	if (!drone.initialize())
	{
		ROS_FATAL("Failed to initialize drone!");
		exit(1);
	}
	
	while (ros::ok())
	{
		auto [video_frames, telemetry_frames] = drone.poll_data();
		if (video_frames.size() > 0)
		{
			auto& frame = video_frames.back();

			if (frame.timestamp < 0)
			{
				ROS_WARN("got image frame with negative timestamp, ignoring!");
			}
			else
			{
				cv_img.image = frame.frame;
				cv_img.header.frame_id = "libcopter";
				cv_img.header.stamp = timestamp(frame.timestamp);
				
				image.publish(cv_img.toImageMsg());
			}
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
}
