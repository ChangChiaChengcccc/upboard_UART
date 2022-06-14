#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_datatypes.h>
#include <queue>
#include <geometry_msgs/Pose2D.h>
#ifndef __ROS_THREAD_H__
#define __ROS_THREAD_H__

#if (MAV_SELECT == FOLLOWER)
void ukf_force_callback(geometry_msgs::Point force);
void optitrack_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
class UKF_force_class
{
private:

public:
	ros::NodeHandle n;
	geometry_msgs::Pose2D payload_data;
	ros::Publisher payload_yaw_pub = n.advertise<geometry_msgs::Pose2D>("optitrack_payload_yaw",2);
	void ukf_force_callback(geometry_msgs::Point force);
};

#elif (MAV_SELECT == LEADER)
void controller_force_callback(geometry_msgs::Point force);
#endif

int ros_thread_entry();

#endif
