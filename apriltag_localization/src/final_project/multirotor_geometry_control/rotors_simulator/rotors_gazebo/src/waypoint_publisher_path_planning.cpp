#include <cmath>
#include <fstream>
#include <unistd.h>
#include <iostream>
#include <ros/ros.h>
#include <Eigen/Core>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

using namespace std;

float C = 1;
nav_msgs::Path Point;

void PathCallback(const nav_msgs::Path::ConstPtr &path_msg)
{
	C = C + 1;

	if (C == 3)
	{
		Point.poses = path_msg->poses;
	}
}

void Path_Planning(ros::NodeHandle &nh, ros::Publisher &trajectory_pub, const float DEG_2_RAD, double desired_yaw)
{
	// Path Planning
	ROS_INFO("Waitting for path planner message");

	// Node setting
	ros::Subscriber path_sub;

	// Variable setting
	ros::Rate loop_rate(10);
	Eigen::Vector3d desired_position;
	path_sub = nh.subscribe("/path", 1000, PathCallback);
	trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
	
	float x, y, z, loop;

	// WaitforMessage setting
	boost::shared_ptr<nav_msgs::Path const> sharedEdge;
	sharedEdge = ros::topic::waitForMessage<nav_msgs::Path>("/path");

	if(sharedEdge != NULL)
	{
		ROS_INFO("Getting path planner message");
	}

	// Retrieve waypoints 
	while (ros::ok())
	{
		ros::spinOnce();

		if (C == 3)
		{
			break;
		}
	}

	loop = Point.poses.size();

	for(int i = 1; i <= loop; i++)
	{
		if (loop - i != 0)
		{
			x = 1*Point.poses[loop-i].pose.position.x;
			y = 1*Point.poses[loop-i].pose.position.y;
			z = 10;
		}
		else
		{
			x = 1*Point.poses[loop-i].pose.position.x;
			y = 1*Point.poses[loop-i].pose.position.y;
			z = 15;
		}

		desired_position << x, y, z;

		mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);

		trajectory_pub.publish(trajectory_msg);

		loop_rate.sleep();
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "waypoint_publisher");
	ros::NodeHandle nh;
	ros::Publisher trajectory_pub;

	trajectory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

	ROS_INFO("Started waypoint_publisher.");

	ros::V_string args;
	ros::removeROSArgs(argc, argv, args);

	double delay;

	if (args.size() == 5)
	{
		delay = 1.0;
	} 
	else if (args.size() == 6)
	{
		delay = std::stof(args.at(5));
	}
	else
	{
		ROS_ERROR("Usage: waypoint_publisher <x> <y> <z> <yaw_deg> [<delay>]\n");
		return -1;
	}

	const float DEG_2_RAD = M_PI / 180.0;

	trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
	trajectory_msg.header.stamp = ros::Time::now();

	Eigen::Vector3d desired_position(std::stof(args.at(1)), std::stof(args.at(2)), std::stof(args.at(3)));

	double desired_yaw = std::stof(args.at(4)) * DEG_2_RAD;

	mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
	                desired_yaw, &trajectory_msg);

	// Wait for some time to create the ros publisher.
	ros::Duration(delay).sleep();

	while (trajectory_pub.getNumSubscribers() == 0 && ros::ok())
	{
		ROS_INFO("There is no subscriber available, trying again in 1 second.");
		ros::Duration(1.0).sleep();
	}

	ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",
	         nh.getNamespace().c_str(),
	         desired_position.x(),
	         desired_position.y(),
	         desired_position.z());

	trajectory_pub.publish(trajectory_msg);

	sleep(2);

	Path_Planning(nh, trajectory_pub, DEG_2_RAD, desired_yaw);

	return 0;
}

/*
#include <string>
#include <vector> 
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>

using namespace std;
float C = 1;
nav_msgs::Path Point;

void Callback(const nav_msgs::Path::ConstPtr &msg)
{
	cout << C << endl;
	C = C + 1;

	if (C == 3)
	{
		Point.poses = msg->poses;
	}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Retriever");

  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<nav_msgs::Path>("/RPath", 1000);
  ros::Subscriber sub = n.subscribe("/path", 1000, Callback);

  while (ros::ok())
  {
	ros::spinOnce();

	if (C == 3)
	{
		break;
	}
  }

  cout << Point.poses.size() << endl;
  pub.publish(Point);

  return 0;
}
*/

/*
float C = 1;
nav_msgs::Path Point;

void PathCallback(const nav_msgs::Path::ConstPtr &msg1)
{
	cout << "C: " << C << endl;
	C = C + 1;

	if (C == 3)
	{
		Point.poses = msg1->poses;
	}
}
*/

/*
// Path Planning
ROS_INFO("Waitting for path planner message");

ros::Subscriber path_sub = nh.subscribe("/path", 1000, PathCallback);

boost::shared_ptr<nav_msgs::Path const> sharedEdge;
sharedEdge = ros::topic::waitForMessage<nav_msgs::Path>("/path", nh);

if(sharedEdge != NULL)
{
	ROS_INFO("Getting path planner message");
	// Point = *sharedEdge;
}

while (ros::ok())
{
	ros::spinOnce();

	if (C == 3)
	{
		break;
	}
}

ros::Rate loop_rate(10);

int count = Point.poses.size();
cout << count << endl;

for(int i = 1; i <= count; i++)
{
	if (count - i != 0)
	{
		x = 1*Point.poses[count-i].pose.position.x;
		y = 1*Point.poses[count-i].pose.position.y;
		z = 1;
	}
	else
	{
		x = 1*Point.poses[count-i].pose.position.x;
		y = 1*Point.poses[count-i].pose.position.y;
		z = 15;
	}

	desired_position << x, y, z;

	mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);

	trajectory_pub.publish(trajectory_msg);

	loop_rate.sleep();
}
*/