#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_datatypes.h>
#include <queue>
#ifndef __ROS_THREAD_H__
#define __ROS_THREAD_H__

#if (MAV_SELECT == FOLLOWER) && (MAV_SELECT!=LEADER)
void ukf_force_callback(geometry_msgs::Point force);
void optitrack_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
#endif

#if (MAV_SELECT == LEADER)
void controller_force_callback(geometry_msgs::Point force);
#endif

int ros_thread_entry();

#endif
