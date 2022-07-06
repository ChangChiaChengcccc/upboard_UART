#include "ros/ros.h"
#include<thread>
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "serial.hpp"
#include <sstream>
#include "data_receive.h"

using namespace std;

float ukf_efficiency[4] = {1};

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

void state_cb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
	ukf_efficiency[0] = msg->data[15];  
	ukf_efficiency[1] = msg->data[16];  
	ukf_efficiency[2] = msg->data[17];  
	ukf_efficiency[3] = msg->data[18];  
	/*
	cout << "ukf_efficiency[0]: " << ukf_efficiency[0] << endl;
	cout << "ukf_efficiency[1]: " << ukf_efficiency[1] << endl;
	cout << "ukf_efficiency[2]: " << ukf_efficiency[2] << endl;
	cout << "ukf_efficiency[3]: " << ukf_efficiency[3] << endl << endl;
	*/
//ROS_INFO("I heard: [%s]", msg->data.c_str());
	send_pose_to_serial(ukf_efficiency);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ukf_node");
	serial_init((char *)"/dev/ttyUSB0", 230400);
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/ukf_estimated_state", 10, state_cb);

	

	int count = 0;
	//std::thread thread_talker(haha,&n);	//test
	//std::thread thread_listener(hehe,&n);	//test
	std::thread thread_receiver(data_process,&n); //get_imu data from stm32
	ros::Rate loop_rate(40);
	ros::spin();


  return 0;
}
