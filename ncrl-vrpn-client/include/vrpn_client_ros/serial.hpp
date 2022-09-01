#ifndef __SERIAL_HPP__
#define __SERIAL_HPP__

void reg_serial_with_marker(int id, char *port_name, int baudrate);
void send_pose_to_serial(char *tracker_name, float pos_x_cm, float pos_y_cm, float pos_z_cm,
			 float quat_x, float quat_y, float quat_z, float quat_w);

#endif
