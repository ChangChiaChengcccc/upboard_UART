#include "ros/ros.h"
#include<thread>
#include "std_msgs/String.h"
#include "serial.hpp"
#include <sstream>
#include "data_receive.h"

using namespace std;

void haha(ros::NodeHandle *n){
	ros::Publisher chatter_pub = n->advertise<std_msgs::String>("chatter", 1000);

	int count = 0;
	ros::Rate loop_rate(10);
	std_msgs::String msg;
	while (ros::ok())
	{

		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();

		ROS_INFO("%s", msg.data.c_str());
		chatter_pub.publish(msg);
	//	ros::spinOnce();

		loop_rate.sleep();
		++count;
	}
}
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void hehe(ros::NodeHandle *n){
	ros::Subscriber sub = n->subscribe("chatter", 1000, chatterCallback);
	ros::spin();
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "ukf_node");
	serial_init((char *)"/dev/ttyUSB0", 115200);
	ros::NodeHandle n;

	ros::Rate loop_rate(10);

	int count = 0;

	//std::thread thread_talker(haha,&n);	//test
	//std::thread thread_listener(hehe,&n);	//test
	std::thread thread_receiver(data_process,&n); //get_imu data from stm32

	//thread_talker.join();
	//thread_listener.join();
	thread_receiver.join();

	while (ros::ok())
	{
	}


  return 0;
}
