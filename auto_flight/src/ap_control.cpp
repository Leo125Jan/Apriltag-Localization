#include <vector>
#include <cstdlib>
#include <iostream>
#include <unistd.h>
#include "ros/ros.h"
#include "auto_flight/ncrl_link.h"
#include <geometry_msgs/PoseStamped.h>

using namespace std;

int num;
vector<float> x_pos;
vector<float> y_pos;
vector<float> z_pos;

bool check = false;

void get_waypoint(void)
{
	if (ros::param::get("ap_control/num_waypoint", num))
	{
		ROS_INFO("Get num_waypoint");
	}
	else
	{
		ROS_WARN("Didn't find num_waypoint");
	}

	if (ros::param::get("ap_control/x_pos", x_pos))
	{
		ROS_INFO("Get x position");

		if (x_pos.size() != num)
		{
        	ROS_WARN("Wrong x_pos values.");
        	check = true;
    	}
	}
    else
    {
        ROS_WARN("Didn't find x_pos");
    }

    if (ros::param::get("ap_control/y_pos", y_pos))
    {
    	ROS_INFO("Get y position");

		if (y_pos.size() != num)
		{
        	ROS_WARN("Wrong y_pos values.");
        	check = true;
    	}
    }
    else
    {
        ROS_WARN("Didn't find y_pos");
    }

    if (ros::param::get("ap_control/z_pos", z_pos))
    {
    	ROS_INFO("Get z position");

		if (z_pos.size() != num)
		{
        	ROS_WARN("Wrong y_pos values.");
        	check = true;
    	}
    }
    else
    {
        ROS_WARN("Didn't find z_pos");
    }
}

float ode_x, ode_y, ode_z;
geometry_msgs::PoseStamped self_ode;

void OdeCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	self_ode = *msg;

	ode_x = self_ode.pose.position.x;
	ode_y = self_ode.pose.position.y;
    ode_z = self_ode.pose.position.z;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "waypoint_control");
	ros::NodeHandle nh;

	get_waypoint();

	if (check);
	{
		ros::shutdown();
	}

	// Varaible Configuration
	ros::Publisher wp_pub = nh.advertise<auto_flight::ncrl_link>("/pc_to_pixhawk", 100);
	ros::Subscriber ode_sub = nh.subscribe("/vrpn_client_node/MAV1/pose", 1000, OdeCallback);

	auto_flight::ncrl_link command;

    // Take off
    ROS_INFO("Take off mode");
    sleep(2);

    command.mode = 1;
    command.data1 = 0;
	command.data2 = 0;
	command.data3 = 0;

    wp_pub.publish(command);

    // Go to target
    ROS_INFO("Waypoint mode");
    sleep(2);

    int segment = 0;
    float d, err = 0.1;

    while(ros::ok())
    {
    	ros::spinOnce();

		command.mode = 2;
    	command.data1 = x_pos[segment];
    	command.data2 = y_pos[segment];
    	command.data3 = z_pos[segment];

    	if (abs(ode_x - x_pos[segment]) > err || abs(ode_y - y_pos[segment]) > err)
    	{
    		wp_pub.publish(command);
    		sleep(0.1);
    	}
    	else
    	{
    		if (segment < num)
	    	{
	    		segment++;
	    	}
	    	else
	    	{
	    		break;
	    	}
    	}
    }

    // Land
    ROS_INFO("Land mode");
    sleep(2);

    command.mode = 3;
    command.data1 = 0;
	command.data2 = 0;
	command.data3 = 0;

	wp_pub.publish(command);

    ROS_INFO("Mission Complete");
    sleep(2);

    ros::shutdown();

	return 0;
}