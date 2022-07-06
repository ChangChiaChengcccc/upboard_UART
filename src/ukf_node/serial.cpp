#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <inttypes.h>
#include <string>
#include "ros/ros.h"
#include <queue>

using namespace std;


#define EFFICIENCY_SERIAL_MSG_SIZE 19


int serial_fd = 0;

void serial_init(char *port_name, int baudrate)
{
	//open the port
	serial_fd = open(port_name, O_RDWR | O_NOCTTY /*| O_NONBLOCK*/);

	if(serial_fd == -1) {
		ROS_FATAL("Failed to open the serial port.");
		exit(0);
	}

	//config the port
	struct termios options;

	tcgetattr(serial_fd, &options);

	options.c_cflag = CS8 | CLOCAL | CREAD;
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;

	switch(baudrate) {
	case 9600:
		options.c_cflag |= B9600;
		break;
	case 57600:
		options.c_cflag |= B57600;
		break;
	case 115200:
		options.c_cflag |= B115200;
		break;
	case 230400:
		options.c_cflag |= B230400;
		break;
	default:
		ROS_FATAL("Unknown baudrate. try 9600, 57600, 115200, 230400 or check \"serial.cpp\".");
		exit(0);
	}

	tcflush(serial_fd, TCIFLUSH);
	tcsetattr(serial_fd, TCSANOW, &options);
}
int check_rigid_body_name(char *name, int *id)
{
	char tracker_id_s[100] = {0};

	if(name[0] == 'M' && name[1] == 'A' && name[2] == 'V') {
		strncpy(tracker_id_s, name + 3, strlen(name) - 3);
	}

	//ROS_INFO("%s -> %s", name, tracker_id_s);

	char *end;
	int tracker_id = std::strtol(tracker_id_s, &end, 10);
	if (*end != '\0' || end == tracker_id_s) { //FIXME
		ROS_FATAL("Invalid tracker name %s, correct format: MAV + number, e.g: MAV1", name);
		return 1;
	}

	*id = tracker_id;

	return 0;
}


void serial_puts(char *s, size_t size)
{
	write(serial_fd, s, size);
}

#define FORCE_CHECKSUM_INIT_VAL 0
static uint8_t generate_force_checksum_byte(uint8_t *payload, int payload_count)
{
	uint8_t result = FORCE_CHECKSUM_INIT_VAL;

	int i;
	for(i = 0; i < payload_count; i++)
		result ^= payload[i];

	return result;
}


void send_pose_to_serial(float *send_to_serial_msg)
{

	float e1 = send_to_serial_msg[0];
	float e2 = send_to_serial_msg[1];
	float e3 = send_to_serial_msg[2];
	float e4 = send_to_serial_msg[3];

	char msg_buf[EFFICIENCY_SERIAL_MSG_SIZE] = {0};
	int msg_pos = 0;

	/* reserve 2 for start byte and checksum byte as header */
	msg_buf[msg_pos] = '@'; //start byte
	msg_pos += sizeof(uint8_t);
	msg_buf[msg_pos] = 0; // checksum
	msg_pos += sizeof(uint8_t);

	/* pack payloads */
	// efficiency
	memcpy(msg_buf + msg_pos, &e1, sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &e2, sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &e3, sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &e4, sizeof(float));
	msg_pos += sizeof(float);

   	msg_buf[msg_pos] = '+'; //end byte
	msg_pos += sizeof(uint8_t);

	/* generate and fill the checksum field */
	msg_buf[1] = generate_force_checksum_byte((uint8_t *)&msg_buf[2], EFFICIENCY_SERIAL_MSG_SIZE - 3);

	serial_puts(msg_buf, EFFICIENCY_SERIAL_MSG_SIZE);
}

int serial_getc(char *c)
{
	return read(serial_fd, c, 1);
}
