#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#ifndef __IMU_THREAD_H__
#define __IMU_THREAD_H__

#define IMU_SERIAL_MSG_SIZE 103
#define IMU_CHECKSUM_INIT_VAL 0
typedef struct {
	volatile int buf_pos;	

	uint8_t buf[200];

	float pos_enu[3];

	float vel_enu[3];

	float acc_enu[3];

	float gyro[3];

	float f1_cmd;
	float f2_cmd;
	float f3_cmd;
	float f4_cmd;

	float RotMat_arr[9];
} imu_t ;

void ukf_force_cb(geometry_msgs::Point force);

void controller_force_cb(geometry_msgs::Point force);

double calc_deviation(double*);

uint8_t generate_imu_checksum_byte(uint8_t *, int);

int data_decode(uint8_t *);

void imu_buf_push(uint8_t);

int data_process(ros::NodeHandle *);

#endif
