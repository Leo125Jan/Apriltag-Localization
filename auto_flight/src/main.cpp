#include<iostream>
#include<thread>
#include"ros/ros.h"
#include<serial.hpp>
#include"thread/main_thread.h"

using namespace std;

main(int argc ,char **argv){

	ros::init(argc,argv,"ncrl_auto_flght");
	ros::Time::init();
	
	cout << "main start" << endl;           
	serial_init((char *)"/dev/ttyUSB1", 115200);
	std::thread receive_thread(receive_thread_entry);
	std::thread send_thread(send_thread_entry);
	send_thread.join();
	receive_thread.join();

	return 0;
}
