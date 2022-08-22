#ifndef __SERIAL_HPP__
#define __SERIAL_HPP__

void serial_init(char *port_name, int baudrate);

void send_pose_to_serial(char mode, char aux_info, float data1, float data2, float data3, float data4);
int serial_getc(char *c);

#endif
