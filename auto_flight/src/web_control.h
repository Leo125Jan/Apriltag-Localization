#ifndef __WEB_CONTROL_H__
#define __WEB_CONTROL_H__

#include <vector>
#include <cstdlib>
#include <iostream>
#include <unistd.h>
#include "ros/ros.h"
#include <std_msgs/Int8.h>
#include <geometry_msgs/Pose.h>
#include "auto_flight/ncrl_link.h"
#include <geometry_msgs/PoseStamped.h>

using namespace std;

class WC
{
	public:

		WC(ros::NodeHandle* nodehandle);

		void control_phase_Callback(const std_msgs::Int8::ConstPtr &msg);
		void OdeCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
		void flight_destination_Callback(const geometry_msgs::Pose::ConstPtr &msg);

		void Web_Command_Loop(void);

	private:

		ros::Publisher wp_pub;
		ros::Subscriber ode_sub;
		ros::Subscriber control_phase_sub;
		ros::Subscriber flight_destination_sub;
		
		auto_flight::ncrl_link command;
		geometry_msgs::Pose flight_command;
		geometry_msgs::PoseStamped self_ode;

		int web_c;
		bool take_off_c;
		float ode_x, ode_y, ode_z, err, temp_x, temp_y, temp_z;
};

WC::WC(ros::NodeHandle *nh)
{
	wp_pub = nh->advertise<auto_flight::ncrl_link>("/pc_to_pixhawk", 10);
	ode_sub = nh->subscribe("/vrpn_client_node/MAV1/pose", 10, &WC::OdeCallback, this);
	control_phase_sub = nh->subscribe("/control_phase", 1, &WC::control_phase_Callback, this);
	flight_destination_sub = nh->subscribe("/flight_destination", 10, &WC::flight_destination_Callback, this);

	// Initialization
	ROS_INFO("Initialization");

	flight_command.position.x = 0;
	flight_command.position.y = 0;
	flight_command.position.z = 0;

	command.mode = '0';
    command.data1 = 0;
	command.data2 = 0;
	command.data3 = 0;

	web_c = 0;
	err = 0.2;
	temp_x = 0;
	temp_y = 0;
	temp_z = 0;
	ode_x = 0;
	ode_y = 0;
    ode_z = 0;
	take_off_c = false;
}

void WC::OdeCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	self_ode = *msg;

	ode_x = self_ode.pose.position.x;
	ode_y = self_ode.pose.position.y;
    ode_z = self_ode.pose.position.z;

	cout << ode_x << endl;
	cout << ode_y << endl;
	cout << ode_z << endl;
}

void WC::control_phase_Callback(const std_msgs::Int8::ConstPtr &msg)
{
	web_c = msg->data;
}

void WC::flight_destination_Callback(const geometry_msgs::Pose::ConstPtr &msg)
{
	flight_command = *msg;
}

void WC::Web_Command_Loop(void)
{
	ros::Rate loop_rate(80);

	switch (web_c)
	{
		case 0:

			// Land
		    ROS_INFO("Stop mode");

		    command.mode = '3';
		    command.data1 = 0;
			command.data2 = 0;
			command.data3 = 0;

			wp_pub.publish(command);

			take_off_c = false;

			break;

		case 1:

			if (take_off_c)
			{
				ROS_INFO("Hovering");
			}
			else
			{
			    // Take off
			    ROS_INFO("Take off mode");

			    command.mode = '1';
			    command.data1 = 0;
				command.data2 = 0;
				command.data3 = 0;

				wp_pub.publish(command);

				take_off_c = true;
			}

			break;

		case 2:

		    // Return
		    ROS_INFO("Return mode");

		    command.mode = '2';
	    	command.data1 = 0;
	    	command.data2 = 0;
	    	command.data3 = 0;

	    	wp_pub.publish(command);

			break;

		case 3:

		    // Position
		    ROS_INFO("Position mode");

		    command.mode = '2';

	    	while (web_c == 3)
	    	{
	    		ros::spinOnce();

		    	temp_x = flight_command.position.x;
		    	temp_y = flight_command.position.y;
		    	temp_z = flight_command.position.z;

	    		if (abs(ode_x - temp_x) > err || abs(ode_y - temp_y) > err)
				{

					ROS_INFO("Moving");

			    	command.data1 = temp_x;
			    	command.data2 = temp_y;
			    	command.data3 = temp_z;

			    	cout << command.data1 << endl;
			    	cout << command.data2 << endl;
			    	cout << command.data3 << endl;

			    	wp_pub.publish(command);
				}
				else
				{
					ROS_INFO("Arrival");
				}

				loop_rate.sleep();
	    	}

			break;

		default:

			ROS_INFO("default");

		    command.mode = '0';
		    command.data1 = 0;
			command.data2 = 0;
			command.data3 = 0;

			wp_pub.publish(command);

			break;
	}
}

#endif