#ifndef __SERIAL_HPP__
#define __SERIAL_HPP__
#include <queue>
void serial_init(char *port_name, int baudrate);
// void send_pose_to_serial(std::queue<float> send_to_stm32);
#if (MAV_SELECT == LEADER)
void send_pose_to_serial(float force_x, float force_y, float force_z);
#endif

#if (MAV_SELECT == FOLLOWER)
void send_pose_to_serial(float force_x, float force_y, float force_z, float payload_yaw);
#endif
int serial_getc(char *c);

#endif
