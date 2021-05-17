#include "geometry_msgs/Point.h"
#include <queue>
#ifndef __ROS_THREAD_H__
#define __ROS_THREAD_H__
void ukf_force_callback(geometry_msgs::Point force);
void controller_force_callback(geometry_msgs::Point force);

int ros_thread_entry();

#endif
