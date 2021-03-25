#include "geometry_msgs/Point.h"    
#ifndef __ROS_THREAD_H__
#define __ROS_THREAD_H__
void force_callback(geometry_msgs::Point force); 

int ros_thread_entry();

#endif
