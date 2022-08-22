#ifndef __MAIN_THREAD__
#define __MAIN_THREAD__

#include "auto_flight/ncrl_link.h"

// pc -> px4
#define NCRL_LINK_SERIAL_MSG_SIZE 22
// for check
#define NCRL_LINK_CHECKSUM_INIT_VAL 19
// px4->pc
#define NCRL_LINK_MSG_SIZE 21

#define MASS 1.39f
#define GRAVITY 9.8f


typedef struct {
	
	float acc[3];

	float gyrop[3];

	volatile int buf_pos;

	double deviation_acc;	


	char mode;
	char aux_info;
	float data1;
	float data2;
	float data3;
	float data4;

	uint8_t buf[44];


} ncrl_link_t;

class NCRL_LINK{

private :

	ros::NodeHandle n;
	ros::Publisher pub;

public:

	NCRL_LINK();

	uint8_t generate_ncrl_link_checksum_byte(uint8_t *, int);

	int ncrl_link_decode(uint8_t *);

	void ncrl_link_buf_push(uint8_t);

	void publisher();

	ncrl_link_t rx_data;

};

int receive_thread_entry();

int send_thread_entry();

#endif
