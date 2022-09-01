#include <iostream>
#include "ros/ros.h"
#include <vrpn_client_ros/serial_cal.hpp>
#include <nav_msgs/Odometry.h>

using namespace std;

float cal_x, cal_y, cal_z, cal_w, cal_i, cal_j, cal_k;

void Calibration_Callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  cal_x = (*msg).pose.pose.position.x;
  cal_y = (*msg).pose.pose.position.y;
  cal_z = (*msg).pose.pose.position.z;

  cal_w = (*msg).pose.pose.orientation.w;
  cal_i = (*msg).pose.pose.orientation.x;
  cal_j = (*msg).pose.pose.orientation.y;
  cal_k = (*msg).pose.pose.orientation.z;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "cal_pub");
	ros::NodeHandle nh;
	ros::Rate loop_rate(400);

	ros::Subscriber cal_sub = nh.subscribe("/odometry/filtered", 100, Calibration_Callback);

	serial_init((char *)"/dev/GPS", 115200);

	while (ros::ok())
	{
		ros::spinOnce();

		send_pose_to_serial(cal_x, cal_y, cal_z, cal_i, cal_j, cal_k, cal_w);

		loop_rate.sleep();
	}

	return 0;
}