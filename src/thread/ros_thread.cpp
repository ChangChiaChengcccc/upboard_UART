#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include "upboard_ukf/serial.hpp"
#include "ros_thread.h"
#include <mutex>
#include "geometry_msgs/Point.h"
#include <queue>
#include <tf/transform_datatypes.h>
#include "proj_conf.h"

using namespace std;

mutex ros_mutex;
queue<float> send_to_stm32;
float ground_truth_yaw;

//#if (MAV_SELECT == FOLLOWER)
void ukf_force_callback(geometry_msgs::Point force)
{
	// send_to_stm32.push(force.x);
	// send_to_stm32.push(force.y);
	// send_to_stm32.push(force.z);
	// send_pose_to_serial(send_to_stm32);
	// send_to_stm32 = queue<float>();

#if (MAV_SELECT == FOLLOWER)
	cout << "Hehe" << endl;
#pragma message("I'm follower!")
#else
	cout << "Haha" << endl;
#pragma message("I'm leader!")
#endif
	std::cout << ground_truth_yaw << std::endl;
	//send_pose_to_serial(force.x, force.y, force.z, ground_truth_yaw);
	//ROS_INFO_STREAM("MAV is " << MAV_);
	float follower_send_to_serial[5] = {4.0f, float(force.x), float(force.y), float(force.z), ground_truth_yaw};
	send_pose_to_serial(follower_send_to_serial);
}

void optitrack_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	//cout << "in" << endl;
    geometry_msgs::PoseStamped optitrack_data;
    optitrack_data = *msg;
	double quaternion_w, quaternion_x, quaternion_y, quaternion_z;
	double payload_roll, payload_yaw, payload_pitch;
	quaternion_x = optitrack_data.pose.orientation.x;
	quaternion_y = optitrack_data.pose.orientation.y;
	quaternion_z = optitrack_data.pose.orientation.z;
	quaternion_w = optitrack_data.pose.orientation.w;
	tf::Quaternion quaternion(quaternion_x, quaternion_y, quaternion_z, quaternion_w);
	tf::Matrix3x3(quaternion).getRPY(payload_roll, payload_pitch, payload_yaw);
	ground_truth_yaw = payload_yaw;
}
//#endif

#if (MAV_SELECT == LEADER)
void controller_force_callback(geometry_msgs::Point force)
{
	// send_to_stm32.push(force.x);
	// send_to_stm32.push(force.y);
	// send_to_stm32.push(force.z);
	// send_pose_to_serial(send_to_stm32);
	// send_to_stm32 = queue<float>();
	// send_pose_to_serial(force.x, force.y, force.z);
	float leader_send_to_serial[4] = {3.0f, float(force.x), float(force.y), float(force.z)};
	//send_pose_to_serial(leader_send_to_serial);
}
#endif

int ros_thread_entry(){
	ros::NodeHandle n;
#if (MAV_SELECT == LEADER)
	ros::Subscriber ctrl_sub = n.subscribe("/controller_force",1000,controller_force_callback);

#endif
	ros::Subscriber optitrack_sub = n.subscribe("/vrpn_client_node/payload/pose",1000, optitrack_callback);
	ros::Subscriber ukf_sub = n.subscribe("force_estimate",1000,ukf_force_callback);

	ros::spin();
	return 0;
}
