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

void ukf_force_callback(geometry_msgs::Point force)
{
	// send_to_stm32.push(force.x);
	// send_to_stm32.push(force.y);
	// send_to_stm32.push(force.z);
	// send_pose_to_serial(send_to_stm32);
	// send_to_stm32 = queue<float>();
	send_pose_to_serial(force.x, force.y, force.z);
}

void controller_force_callback(geometry_msgs::Point force)
{
	// send_to_stm32.push(force.x);
	// send_to_stm32.push(force.y);
	// send_to_stm32.push(force.z);
	// send_pose_to_serial(send_to_stm32);
	// send_to_stm32 = queue<float>();
	send_pose_to_serial(force.x, force.y, force.z);
}

int ros_thread_entry(){
	ros::NodeHandle n;
	std::string MAV = "";
	n.getParam("MAV", MAV);
	if(MAV == "leader")
	{
		ros::Subscriber ctrl_sub = n.subscribe("/controller_force",1000,controller_force_callback);
	}
	else if(MAV == "follower")
	{
		ros::Subscriber ukf_sub = n.subscribe("force_estimate",1000,ukf_force_callback);
	}
	else
	{
		ROS_INFO("Error! Please input the MAV type ex.leader, follower");
	}
	ros::spin();
	return 0;
}
