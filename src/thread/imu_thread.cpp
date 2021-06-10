#include <iostream>
#include <upboard_ukf/serial.hpp>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Int32.h>
#include <imu_thread.h>
#include "ros/ros.h"
#include <mutex>
#include <cmath>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include "upboard_ukf/serial.hpp"
#include "ros_thread.h"
#include "proj_conf.h"

using namespace std;

float ukf_force[3] = {0.0f};
float controller_force[3] = {0.0f};

mutex imu_mutex;
imu_t imu;
float imu_average[3] = {0,0,-9.8};
int N = 100;
double calc_deviation(float* x){
	double sum = 0;
	double average = 0;
	double result = IMU_CHECKSUM_INIT_VAL;
	for(int i = 0 ; i<N ; i++ ){
		sum += x[i];
	}
	average = sum / N ;
	sum =0;
	for(int i = 0 ; i<N ; i++){
		sum += pow((x[i]-average),2);
	}
	result = sqrt(sum/N);
	return result;
}

void send_force_to_imu_thread(float* output_force);

uint8_t generate_imu_checksum_byte(uint8_t *payload, int payload_count)
{
	uint8_t result = 0;

	int i;
	for(i = 0; i < payload_count; i++)
		result ^= payload[i];

	return result;
}

int imu_decode(uint8_t *buf){
	static float x_array_uart[100];
	uint8_t recv_checksum = buf[1];
	uint8_t checksum = generate_imu_checksum_byte(&buf[2], IMU_SERIAL_MSG_SIZE - 3);
	if(checksum != recv_checksum) {
		printf("oh no grabage message!\n");
		return 1; //error detected
	}

	memcpy(&imu.thrust, &buf[2], sizeof(float));

	// float enu_acc_x, enu_acc_y, enu_acc_z;

	/* swap the order of quaternion to make the frame consistent with ahrs' rotation order */
	memcpy(&imu.gyrop[0], &buf[6], sizeof(float));
	memcpy(&imu.gyrop[1], &buf[10], sizeof(float));
	memcpy(&imu.gyrop[2], &buf[14], sizeof(float));

	return 0;
}

void imu_buf_push(uint8_t c)
{
	if(imu.buf_pos >= IMU_SERIAL_MSG_SIZE) {
		/* drop the oldest data and shift the rest to left */
		int i;
		for(i = 1; i < IMU_SERIAL_MSG_SIZE; i++) {
			imu.buf[i - 1] = imu.buf[i];
		}

		/* save new byte to the last array element */
		imu.buf[IMU_SERIAL_MSG_SIZE - 1] = c;
		imu.buf_pos = IMU_SERIAL_MSG_SIZE;
	} else {
		/* append new byte if the array boundary is not yet reached */
		imu.buf[imu.buf_pos] = c;
		imu.buf_pos++;
	}
}

void ukf_force_cb(geometry_msgs::Point force)
{
	ukf_force[0] = force.x;
	ukf_force[1] = force.y;
	ukf_force[2] = force.z;
}

void controller_force_cb(geometry_msgs::Point force)
{
	controller_force[0] = force.x;
	controller_force[1] = force.y;
	controller_force[2] = force.z;
}

int imu_thread_entry(){
	sensor_msgs::Imu IMU_data;
	geometry_msgs::WrenchStamped thrust_data;
	ros::NodeHandle n;
#if (MAV_SELECT == FOLLOWER)
	ros::Subscriber sub = n.subscribe("force_estimate",1000,ukf_force_cb);
#elif (MAV_SELECT == LEADER)
	ros::Subscriber ctrl_sub = n.subscribe("/controller_force",1000,controller_force_cb);
#endif
	ros::Publisher omega_pub = n.advertise<sensor_msgs::Imu>("imu/data_raw", 5);
	ros::Publisher thrust_pub = n.advertise<geometry_msgs::WrenchStamped>("/rotor_all_ft", 5);
	ros::Publisher yaw_pub = n.advertise<geometry_msgs::Pose2D>("/stm32_payload_yaw",5);
	char c;
	imu.buf_pos = 0;
	int count1 =0;
	while(ros::ok()){
		if(serial_getc(&c) != -1) {
			imu_buf_push(c);
			/*
			if(c == '-')
				count1 = 1;
			else if(c == '+') {
				count1++;
				printf("count : %d\n",count1);}
			else
				count1++;

			*/

			if(imu.buf[0]=='@' && imu.buf[IMU_SERIAL_MSG_SIZE - 1 ] == '+')
			{
				for(int i =0;i<IMU_SERIAL_MSG_SIZE;i++)
					cout << "s";
#if (MAV_SELECT == FOLLOWER)
				printf("\t UKF estimated force  x: %f  y: %f  z: %f\n", ukf_force[0], ukf_force[1], ukf_force[2]);
#elif (MAV_SELECT == LEADER)
				printf("\t controller force  x: %f  y: %f  z: %f\n", controller_force[0], controller_force[1], controller_force[2]);
#endif
				if(imu_decode(imu.buf)==0)
				{
					IMU_data.header.stamp = ros::Time::now();
					IMU_data.header.frame_id = "base_link";
					// imu.acc is ned , IMU_data should be enu
					// IMU_data.linear_acceleration.x = imu.thrust;
					thrust_data.wrench.force.z = imu.thrust;
					// IMU_data.linear_acceleration.y = imu.acc[0];
					// IMU_data.linear_acceleration.z = -imu.acc[2];
					IMU_data.angular_velocity.x = imu.gyrop[0]*M_PI/180.0f;
					IMU_data.angular_velocity.y = imu.gyrop[1]*M_PI/180.0f;
					IMU_data.angular_velocity.z = -imu.gyrop[2]*M_PI/180.0f;	//*M_PI/180.0f
					IMU_data.angular_velocity_covariance={1.2184696791468346e-07, 0.0, 0.0, 0.0, 1.2184696791468346e-07, 0.0, 0.0, 0.0, 1.2184696791468346e-07};
					IMU_data.linear_acceleration_covariance={8.999999999999999e-08, 0.0, 0.0, 0.0, 8.999999999999999e-08, 0.0, 0.0, 0.0, 8.999999999999999e-08};

					omega_pub.publish(IMU_data);
					thrust_pub.publish(thrust_data);
				}
			}
		}
	}
}
