#ifndef __SERIAL_HPP__
#define __SERIAL_HPP__

void serial_init(char *port_name, int baudrate);
void send_pose_to_serial(float force_x, float force_y, float force_z);
int serial_getc(char *c);

#endif
