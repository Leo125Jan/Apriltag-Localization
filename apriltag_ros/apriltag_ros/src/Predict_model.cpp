#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>


float i = 0.0;
float j = 0.0;
float k = 0.0;
float w = 0.0;

void OdeCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	i = (*msg).pose.orientation.x;
	j = (*msg).pose.orientation.y;
    k = (*msg).pose.orientation.z;
    w = (*msg).pose.orientation.w;
}


int main(int argc, char** argv)
{
	ROS_INFO("Prediction");
	ros::init(argc, argv, "Prediction");

	ode_sub = nh.subscribe("/vrpn_client_node/MAV1/pose", 1000, OdeCallback);
}