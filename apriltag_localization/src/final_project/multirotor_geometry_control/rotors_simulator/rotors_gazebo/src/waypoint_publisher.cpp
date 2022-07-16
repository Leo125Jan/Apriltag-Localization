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

#include <fstream>
#include <iostream>
#include <cmath>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <unistd.h>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <std_msgs/String.h>
#include <std_msgs/Int64.h>

using namespace std;

void Go_to_waypoint(ros::NodeHandle &nh, ros::Publisher &trajectory_pub, const float DEG_2_RAD, double desired_yaw)
{
	// Go to waypoint
	ROS_INFO("wait for neo11 waypoint message");

	// Node setting
	ros::Publisher detach_pub;

	// Variable setting
	Eigen::Vector3d desired_position;
	trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    detach_pub = nh.advertise<std_msgs::Int64>("/iris/detach", 1000);

    sleep(10);

    // WaitforMessage setting
	boost::shared_ptr<std_msgs::Int64 const> sharedEdge;	
	sharedEdge = ros::topic::waitForMessage<std_msgs::Int64>("/neo11/waypoint");

	if(sharedEdge != NULL)
	{
		ROS_INFO("Get neo11 waypoint message");
	}
    
	float x = 8, y = 8, z = 5;

	desired_position << x, y, z;

	mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);

	trajectory_pub.publish(trajectory_msg);

    sleep(10);

    std_msgs::Int64 Do;
	Do.data = 1;

    for(int i = 0; i < 100; i++)
    {
    	detach_pub.publish(Do);
    }
}

void Go_to_waypoint_2(ros::NodeHandle &nh, ros::Publisher &trajectory_pub, const float DEG_2_RAD, double desired_yaw)
{
	// Go to waypoint
	ROS_INFO("wait for neo11 second waypoint message");

	// Node setting
	ros::Publisher detach_pub_2;

	// Variable setting
	Eigen::Vector3d desired_position;
	trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    detach_pub_2 = nh.advertise<std_msgs::Int64>("/iris2/detach", 1000);

    sleep(10);

    // WaitforMessage setting
	boost::shared_ptr<std_msgs::Int64 const> sharedEdge;	
	sharedEdge = ros::topic::waitForMessage<std_msgs::Int64>("/neo11/waypoint_2");

	if(sharedEdge != NULL)
	{
		ROS_INFO("Get neo11 second waypoint message");
	}
    
	float x = 5, y = 5, z = 5;

	desired_position << x, y, z;

	mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);

	trajectory_pub.publish(trajectory_msg);

    sleep(10);

    std_msgs::Int64 Do_2;
    Do_2.data = 1;

    for(int i = 0; i < 500; i++)
    {
    	detach_pub_2.publish(Do_2);
    }
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "waypoint_publisher");
	ros::NodeHandle nh;
	ros::Publisher trajectory_pub =
	        nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
	                mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

	ROS_INFO("Started waypoint_publisher.");

	ros::V_string args;
	ros::removeROSArgs(argc, argv, args);

	double delay;

	if (args.size() == 5) {
		delay = 1.0;
	} else if (args.size() == 6) {
		delay = std::stof(args.at(5));
	} else {
		ROS_ERROR("Usage: waypoint_publisher <x> <y> <z> <yaw_deg> [<delay>]\n");
		return -1;
	}

	const float DEG_2_RAD = M_PI / 180.0;

	trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
	trajectory_msg.header.stamp = ros::Time::now();

	Eigen::Vector3d desired_position(std::stof(args.at(1)), std::stof(args.at(2)),
	                                 std::stof(args.at(3)));

	double desired_yaw = std::stof(args.at(4)) * DEG_2_RAD;

	mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
	                desired_yaw, &trajectory_msg);

	// Wait for some time to create the ros publisher.
	ros::Duration(delay).sleep();

	while (trajectory_pub.getNumSubscribers() == 0 && ros::ok()) {
		ROS_INFO("There is no subscriber available, trying again in 1 second.");
		ros::Duration(1.0).sleep();
	}

	ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",
	         nh.getNamespace().c_str(),
	         desired_position.x(),
	         desired_position.y(),
	         desired_position.z());

	trajectory_pub.publish(trajectory_msg);

	// sleep(3);

	// Go_to_waypoint(nh, trajectory_pub, DEG_2_RAD, desired_yaw);

 //    sleep(5);

 //    sleep(3);

	// Go_to_waypoint_2(nh, trajectory_pub, DEG_2_RAD, desired_yaw);

 //    sleep(5);

	ros::spinOnce();
	ros::shutdown();

	return 0;
}