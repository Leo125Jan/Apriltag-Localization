/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <cmath>
#include <fstream>
#include <unistd.h>
#include <iostream>
#include <ros/ros.h>
#include <Eigen/Core>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <mav_msgs/Actuators.h>
#include <geometry_msgs/Pose.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

using namespace std;

geometry_msgs::Pose self_ode;

float ode_x, ode_y, ode_z;
float C = 1;

void OdeCallback(const geometry_msgs::Pose::ConstPtr &msg1)
{
	C = C + 1;

	if (C == 3)
	{
		self_ode = *msg1;

		ode_x = self_ode.position.x;
		ode_y = self_ode.position.y;
	    ode_z = self_ode.position.z;
	}
}

void Move_to_Husky(ros::NodeHandle &nh, ros::Publisher &trajectory_pub, const float DEG_2_RAD, double desired_yaw)
{
	// Move to husky
	ROS_INFO("Waitting for Husky position message");

	// Node setting
	ros::Subscriber ode_sub;

	// Variable setting
	ode_sub = nh.subscribe("/firefly/odometry_sensor1/pose", 1000, OdeCallback);

	Eigen::Vector3d desired_position;
	apriltag_ros::AprilTagDetectionArray Husky_pose;
	trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;

	float x, y, z, Real_x, Real_y, Real_z;

	// WaitforMessage setting
	boost::shared_ptr<apriltag_ros::AprilTagDetectionArray const> sharedEdge;	
	sharedEdge = ros::topic::waitForMessage<apriltag_ros::AprilTagDetectionArray>("/tag_detections");

	if(sharedEdge != NULL)
	{
		ROS_INFO("Getting Husky position message");
		Husky_pose = *sharedEdge;
	}

	sleep(1);

	for(int i = 1;i <= 10;i++)
	{
		ros::spinOnce();

		if (C == 3)
		{
			break;
		}
	}
	cout << "ode_x: " << ode_x << ", ode_y: " << ode_y << endl;

	x = Husky_pose.detections[0].pose.pose.pose.position.x;
	y = Husky_pose.detections[0].pose.pose.pose.position.y;
	z = Husky_pose.detections[0].pose.pose.pose.position.z;

	Real_x = -(y + 0.36) + ode_x;
	Real_y = -(x + 2) + ode_y;
	Real_z = z - 5;

	desired_position << Real_x, Real_y, Real_z;

	mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);

	trajectory_pub.publish(trajectory_msg);
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

	Move_to_Husky(nh, trajectory_pub, DEG_2_RAD, desired_yaw);

	return 0;
}

/*
// Move to husky
ROS_INFO("Waitting for Husky position message");

boost::shared_ptr<apriltag_ros::AprilTagDetectionArray const> sharedEdge;
sharedEdge = ros::topic::waitForMessage<apriltag_ros::AprilTagDetectionArray>("/tag_detections", nh);

apriltag_ros::AprilTagDetectionArray Husky_pose;

if(sharedEdge != NULL)
{
	ROS_INFO("Getting Husky position message");
	Husky_pose = *sharedEdge;
}

float x,y,z;
x = Husky_pose.detections[0].pose.pose.pose.position.x;
y = Husky_pose.detections[0].pose.pose.pose.position.y;
z = Husky_pose.detections[0].pose.pose.pose.position.z;

Real_x = -(y + 0.36) + 15.5;
Real_y = -(x + 2) + 10;
Real_z = z - 5;

desired_position << Real_x, Real_y, Real_z;

mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);

trajectory_pub.publish(trajectory_msg);
*/
