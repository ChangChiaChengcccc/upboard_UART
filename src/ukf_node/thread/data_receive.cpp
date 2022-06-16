#include <iostream>
#include <upboard_ukf/serial.hpp>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int32.h>
#include "ros/ros.h"
#include <mutex>
#include <cmath>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include "upboard_ukf/serial.hpp"
#include "data_receive.h"
#include "data_transmit.h"

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

int data_decode(uint8_t *buf){
	static float x_array_uart[100];
	uint8_t recv_checksum = buf[1];
	uint8_t checksum = generate_imu_checksum_byte(&buf[2], IMU_SERIAL_MSG_SIZE - 3);
	if(checksum != recv_checksum) {
		printf("oh no garbage message!\n");
		return 1; //error detected
	}

	memcpy(&imu.pos_enu[0], &buf[2], sizeof(float));
	memcpy(&imu.pos_enu[1], &buf[6], sizeof(float));
	memcpy(&imu.pos_enu[2], &buf[10], sizeof(float));
	memcpy(&imu.gyro[0], &buf[14], sizeof(float));
	memcpy(&imu.gyro[1], &buf[18], sizeof(float));
	memcpy(&imu.gyro[2], &buf[22], sizeof(float));
	memcpy(&imu.f1_cmd, &buf[26], sizeof(float));
	memcpy(&imu.f2_cmd, &buf[30], sizeof(float));
	memcpy(&imu.f3_cmd, &buf[34], sizeof(float));
	memcpy(&imu.f4_cmd, &buf[38], sizeof(float));
	
	memcpy(&imu.RotMat_arr[0], &buf[42], sizeof(float));
	memcpy(&imu.RotMat_arr[1], &buf[46], sizeof(float));
	memcpy(&imu.RotMat_arr[2], &buf[50], sizeof(float));
	memcpy(&imu.RotMat_arr[3], &buf[54], sizeof(float));
	memcpy(&imu.RotMat_arr[4], &buf[58], sizeof(float));
	memcpy(&imu.RotMat_arr[5], &buf[62], sizeof(float));
	memcpy(&imu.RotMat_arr[6], &buf[66], sizeof(float));
	memcpy(&imu.RotMat_arr[7], &buf[70], sizeof(float));
	memcpy(&imu.RotMat_arr[8], &buf[74],sizeof(float));

	
	// float enu_acc_x, enu_acc_y, enu_acc_z;

	/* swap the order of quaternion to make the frame consistent with ahrs' rotation order */
	//memcpy(&imu.gyrop[0], &buf[6], sizeof(float));
	//memcpy(&imu.gyrop[1], &buf[10], sizeof(float));
	//memcpy(&imu.gyrop[2], &buf[14], sizeof(float));
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


int data_process(ros::NodeHandle *n){
	geometry_msgs::Point pos_enu;
	sensor_msgs::Imu gyro;
	geometry_msgs::WrenchStamped f1_cmd, f2_cmd, f3_cmd, f4_cmd;
	std_msgs::Float32MultiArray RotMat;


	ros::Publisher pos_enu_pub = n->advertise<geometry_msgs::Point>("/pos_enu", 5);
	ros::Publisher gyro_pub = n->advertise<sensor_msgs::Imu>("/angular_vel", 5);
	ros::Publisher f1_cmd_pub = n->advertise<geometry_msgs::WrenchStamped>("/f1_cmd", 5);
	ros::Publisher f2_cmd_pub = n->advertise<geometry_msgs::WrenchStamped>("/f2_cmd", 5);
	ros::Publisher f3_cmd_pub = n->advertise<geometry_msgs::WrenchStamped>("/f3_cmd", 5);
	ros::Publisher f4_cmd_pub = n->advertise<geometry_msgs::WrenchStamped>("/f4_cmd", 5);
	ros::Publisher RotMat_pub = n->advertise<std_msgs::Float32MultiArray>("/RotMat_arr", 5);
	char c;
	imu.buf_pos = 0;
	while(ros::ok()){
		if(serial_getc(&c) != -1) {
			imu_buf_push(c);
			
			if(imu.buf[0]=='@' && imu.buf[IMU_SERIAL_MSG_SIZE - 1 ] == '+')
			{
				//for(int i =0;i<IMU_SERIAL_MSG_SIZE;i++)
				//	cout << "s";

				if(data_decode(imu.buf)==0)
				{
					/*
					cout << "imu.pos_enu_x:" << imu.pos_enu[0] << endl;
					cout << "imu.pos_enu_y:" << imu.pos_enu[1] << endl;
					cout << "imu.pos_enu_z:" << imu.pos_enu[2] << endl;

					cout << "gyro_x:" << imu.gyro[0] << endl;
					cout << "gyro_y:" << imu.gyro[1] << endl;
					cout << "gyro_z:" << imu.gyro[2] << endl;

					cout << "f1_cmd:" << imu.f1_cmd << endl;
					cout << "f2_cmd:" << imu.f2_cmd << endl;
					cout << "f3_cmd:" << imu.f3_cmd << endl;
					cout << "f4_cmd:" << imu.f4_cmd << endl;
					
					cout << "imu.RotMat_arr:" << imu.RotMat_arr[0] << endl;
					cout << "imu.RotMat_arr:" << imu.RotMat_arr[4] << endl;
					cout << "imu.RotMat_arr:" << imu.RotMat_arr[8] << endl;
					*/
					
					pos_enu.x = imu.pos_enu[0];
					pos_enu.y = imu.pos_enu[1];
					pos_enu.z = imu.pos_enu[2];
					
					gyro.angular_velocity.x = imu.gyro[0];
					gyro.angular_velocity.y = imu.gyro[1];
					gyro.angular_velocity.z = imu.gyro[2];	

					f1_cmd.wrench.force.z = imu.f1_cmd;
					f2_cmd.wrench.force.z = imu.f2_cmd;
					f3_cmd.wrench.force.z = imu.f3_cmd;
					f4_cmd.wrench.force.z = imu.f4_cmd;
					
					RotMat.data ={
									imu.RotMat_arr[0],imu.RotMat_arr[1],imu.RotMat_arr[2],
									imu.RotMat_arr[3],imu.RotMat_arr[4],imu.RotMat_arr[5],
									imu.RotMat_arr[6],imu.RotMat_arr[7],imu.RotMat_arr[8],
								 };
					
					cout << "decode suceed!" << endl;
					
					//pub & sub
					pos_enu_pub.publish(pos_enu);
					gyro_pub.publish(gyro);
					f1_cmd_pub.publish(f1_cmd);
					f2_cmd_pub.publish(f2_cmd);
					f3_cmd_pub.publish(f3_cmd);
					f4_cmd_pub.publish(f4_cmd);
					RotMat_pub.publish(RotMat);
					

				}
			}
		}
	}
}
