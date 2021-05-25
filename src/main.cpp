#include<iostream>
#include<thread>
#include"ros/ros.h"
#include<upboard_ukf/serial.hpp>
#include"thread/ros_thread.h"
#include"thread/imu_thread.h"
#include "proj_conf.h"

using namespace std;

int main(int argc ,char **argv){
	ros::init(argc,argv,"ukf_to_controller");
	serial_init((char *)"/dev/ttyUSB0", 115200);
	std::thread thread_imu(imu_thread_entry);	//get_imu data from stm32
	std::thread thread_ros(ros_thread_entry);	//push imu data to ROS and recieve position data from ROS

	thread_imu.join();
	thread_ros.join();

	return 0;
}
