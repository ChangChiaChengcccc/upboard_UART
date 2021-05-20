#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include "upboard_ukf/serial.hpp"
#include "ros_thread.h"
#include <mutex>
#include "geometry_msgs/Point.h"
#include <queue>

using namespace std;

mutex ros_mutex;
queue<float> send_to_stm32;
static std::string MAV_;

void ukf_force_callback(geometry_msgs::Point force)
{
	// send_to_stm32.push(force.x);
	// send_to_stm32.push(force.y);
	// send_to_stm32.push(force.z);
	// send_pose_to_serial(send_to_stm32);
	// send_to_stm32 = queue<float>();
	if(MAV_ == "follower")
	{
		send_pose_to_serial(force.x, force.y, force.z);
	}
	//send_pose_to_serial(force.x, force.y, force.z);
	//ROS_INFO_STREAM("MAV is " << MAV_);
}

void controller_force_callback(geometry_msgs::Point force)
{
	// send_to_stm32.push(force.x);
	// send_to_stm32.push(force.y);
	// send_to_stm32.push(force.z);
	// send_pose_to_serial(send_to_stm32);
	// send_to_stm32 = queue<float>();
	if(MAV_ == "leader")
	{
		send_pose_to_serial(force.x, force.y, force.z);
	}
	//send_pose_to_serial(force.x, force.y, force.z);
}

int ros_thread_entry(){
	ros::NodeHandle n;
	n.param<std::string>("MAV", MAV_, "follower");
	//n.getParam("MAV", MAV);
	ros::Subscriber ctrl_sub = n.subscribe("/controller_force",1000,controller_force_callback);
	ros::Subscriber ukf_sub = n.subscribe("force_estimate",1000,ukf_force_callback);
	ros::spin();
	return 0;
}
