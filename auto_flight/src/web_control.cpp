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

// Odometry Track
float ode_x, ode_y, ode_z;
geometry_msgs::PoseStamped self_ode;

void OdeCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	self_ode = *msg;

	ode_x = self_ode.pose.position.x;
	ode_y = self_ode.pose.position.y;
    ode_z = self_ode.pose.position.z;
}

// Web Command Track
int web_c;

void control_phase_Callback(const std_msgs::Int8::ConstPtr &msg)
{

	web_c = msg->data;
}

// Web Waypoint Commmand Track
geometry_msgs::Pose flight_command;

void flight_destination_Callback(const geometry_msgs::Pose::ConstPtr &msg)
{
	flight_command = *msg;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "waypoint_control");
	ros::NodeHandle nh;

	// Ner Setting
	ros::Publisher wp_pub = nh.advertise<auto_flight::ncrl_link>("/pc_to_pixhawk", 100);
	ros::Subscriber ode_sub = nh.subscribe("/vrpn_client_node/MAV1/pose", 1000, OdeCallback);
	ros::Subscriber control_phase_sub = nh.subscribe("/control_phase", 1, control_phase_Callback);
	ros::Subscriber flight_destination_sub = nh.subscribe("/flight_destination", 1000, flight_destination_Callback);

	// Varaible Setting
	ros::Rate loop_rate(200);
	auto_flight::ncrl_link command;

	float err = 0.2;
	bool take_off_c = false;

	float temp_x = 0;
	float temp_y = 0;
	float temp_z = 0;

	// Initialization
	ROS_INFO("Initialization");

	web_c = 0;
	flight_command.position.x = 0;
	flight_command.position.y = 0;
	flight_command.position.z = 0;

	command.mode = '0';
    command.data1 = 0;
	command.data2 = 0;
	command.data3 = 0;

	wp_pub.publish(command);

	sleep(2);

	// Web Command Loop
	while (ros::ok())
	{
		ros::spinOnce();

		cout << web_c << endl;

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

			    	command.data1 = flight_command.position.x;
			    	command.data2 = flight_command.position.y;
			    	command.data3 = flight_command.position.z;

			    	temp_x = flight_command.position.x;
			    	temp_y = flight_command.position.y;
			    	temp_z = flight_command.position.z;

			    	cout << command.data1 << endl;
			    	cout << command.data2 << endl;
			    	cout << command.data3 << endl;

			    	wp_pub.publish(command);

		    		if (abs(ode_x - temp_x) < err && abs(ode_y - temp_y) < err)
					{
						ROS_INFO("Arrival");
					}
					else
					{
						ROS_INFO("Moving");
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

	loop_rate.sleep();
}