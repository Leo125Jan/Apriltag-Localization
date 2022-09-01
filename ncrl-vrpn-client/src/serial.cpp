#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <inttypes.h>
#include <string>
#include "ros/ros.h"

#define MAX_MARKER_CNT 100

int marker_cnt = 0;

using namespace std;

int serial_fd[MAX_MARKER_CNT] = {0};

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

void reg_serial_with_marker(int id, char *port_name, int baudrate)
{
	//open the port
	serial_fd[id] = open(port_name, O_RDWR | O_NOCTTY /*| O_NONBLOCK*/);

	if(serial_fd[id] == -1) {
		ROS_FATAL("Failed to open the serial port.");
		exit(0);
	}

	//config the port
	struct termios options;

	tcgetattr(serial_fd[id], &options);

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
	default:
		ROS_FATAL("Unknown baudrate. try 9600, 57600, 115200 or check \"serial.cpp\".");
		exit(0);
	}

	tcflush(serial_fd[id], TCIFLUSH);
	tcsetattr(serial_fd[id], TCSANOW, &options);

	marker_cnt++;
}

void serial_puts(int id, char *s, size_t size)
{
	write(serial_fd[id], s, size);
}

#define OPTITRACK_CHECKSUM_INIT_VAL 19
static uint8_t generate_optitrack_checksum_byte(uint8_t *payload, int payload_count)
{
	uint8_t result = OPTITRACK_CHECKSUM_INIT_VAL;

	int i;
	for(i = 0; i < payload_count; i++)
		result ^= payload[i];

	return result;
}

#define OPTITRACK_SERIAL_MSG_SIZE 32
void send_pose_to_serial(char *tracker_name, float pos_x_m, float pos_y_m, float pos_z_m,
			 float quat_x, float quat_y, float quat_z, float quat_w)
{
	static double last_execute_time = 0;
	static double current_time;

	int tracker_id;
	if(check_rigid_body_name(tracker_name, &tracker_id)) {
		return;
	}

	if(tracker_id > marker_cnt) {
		ROS_WARN("%s is not registered for serial I/O!", tracker_name);
		return;
	}

	current_time = ros::Time::now().toSec();

	double real_freq = 1.0f / (current_time - last_execute_time); //real sending frequeuncy

	last_execute_time = current_time;

	ROS_INFO("[%fHz] id:%d, position=(x:%.2lf, y:%.2lf, z:%.2lf), "
                 "orientation=(x:%.2lf, y:%.2lf, z:%.2lf, w:%.2lf)",
        	 real_freq, tracker_id,
                 pos_x_m * 100.0f, pos_y_m * 100.0f, pos_z_m * 100.0f,
                 quat_x, quat_y, quat_z, quat_w);

	/*+------------+----------+----+---+---+---+----+----+----+----+----------+
         *| start byte | checksum | id | x | y | z | qx | qy | qz | qw | end byte |
         *+------------+----------+----+---+---+---+----+----+----+----+----------+*/
	char msg_buf[OPTITRACK_SERIAL_MSG_SIZE] = {0};
	int msg_pos = 0;

	/* reserve 2 for start byte and checksum byte as header */
	msg_buf[msg_pos] = '@'; //start byte
	msg_pos += sizeof(uint8_t);
	msg_buf[msg_pos] = 0;
	msg_pos += sizeof(uint8_t);
	msg_buf[msg_pos] = tracker_id;
	msg_pos += sizeof(uint8_t);

	/* pack payloads */
	memcpy(msg_buf + msg_pos, &pos_x_m, sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &pos_y_m, sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &pos_z_m, sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &quat_x, sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &quat_y, sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &quat_z, sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &quat_w, sizeof(float));
	msg_pos += sizeof(float);

        msg_buf[msg_pos] = '+'; //end byte
	msg_pos += sizeof(uint8_t);

	/* generate and fill the checksum field */
	msg_buf[1] = generate_optitrack_checksum_byte((uint8_t *)&msg_buf[3], OPTITRACK_SERIAL_MSG_SIZE - 4);

	serial_puts(tracker_id, msg_buf, OPTITRACK_SERIAL_MSG_SIZE);
}
