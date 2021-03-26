#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include "vins_uart/serial.hpp"
#include "ros_thread.h"
#include <mutex>
#include "geometry_msgs/Point.h"


using namespace std;

mutex ros_mutex;

void force_callback(geometry_msgs::Point force)
{
//	cout << odom.pose.pose.position.x << endl;
	send_pose_to_serial( 
				force.x,
				force.y,
				force.z
			);
}
int ros_thread_entry(){
	
	ros::NodeHandle n;	
	ros::Subscriber sub = n.subscribe("force_estimate",1000,force_callback);
	
	ros::spin();
	
	return 0;
}
