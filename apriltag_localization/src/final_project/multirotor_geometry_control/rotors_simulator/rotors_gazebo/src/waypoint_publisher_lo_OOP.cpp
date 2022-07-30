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
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <std_msgs/String.h>
#include <std_msgs/Int64.h>
#include <mav_msgs/RollPitchYawrateThrust.h>

using namespace std;

const float DEG_2_RAD = M_PI / 180.0;

class WayPointPublisher
{
private:

	trajectory_msgs::MultiDOFJointTrajectoryPoint Point;
	float d_x;
	float d_y;
	float d_z;
	float v_x;
	float v_y;
	float v_z;

	//////////////// Publisher && Subscriber ///////////////

	ros::NodeHandle nh;
	ros::Publisher trajectory_pub;
	ros::Subscriber path_sub;

	////////////////////////////////////////////////////////

	double delay;

	///////////////// Desired pos && traj //////////////////
	trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
	Eigen::Vector3d desired_position;
	double desired_yaw;

public:

	WayPointPublisher();
	void PathCallback(const trajectory_msgs::MultiDOFJointTrajectoryPoint::ConstPtr &msg);
	int setDelay(ros::V_string args);
	void setDesiredPos(double x, double y, double z);
	void setDesiredYaw(double yaw);
	void setDesiredPosYaw(double x, double y, double z, double yaw);
	void setTrajFromPosYaw();
	void setTrajFromPosYaw(double x, double y, double z, double yaw);
	void waitForCreatePublisher();
	void checkSubscriberOfTrajectoryPub();
	ros::NodeHandle getNodeHandler();
	Eigen::Vector3d getDesiredPos();
	void publishTraj();
	void waitForTrajMsg();
	void setTrajFromCallBack();
	
};

WayPointPublisher::WayPointPublisher()
{
	trajectory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
	ROS_INFO("Started waypoint_publisher.");

	trajectory_msg.header.stamp = ros::Time::now();
	path_sub = nh.subscribe("/desired_trajectory", 1000, &WayPointPublisher::PathCallback, this);
}


void WayPointPublisher::PathCallback(const trajectory_msgs::MultiDOFJointTrajectoryPoint::ConstPtr &msg)
{
	Point = *msg;

	d_x = Point.transforms[0].translation.x;
	d_y = Point.transforms[0].translation.y;
	d_z = Point.transforms[0].translation.z;
	v_x = Point.velocities[0].linear.x;
	v_y = Point.velocities[0].linear.y;
	v_z = Point.velocities[0].linear.z;


}

int WayPointPublisher::setDelay(ros::V_string args)
{
	if (args.size() == 5)
		delay = 1.0;

	else if (args.size() == 6) 
		delay = std::stof(args.at(5));

	else 
	{
		ROS_ERROR("Usage: waypoint_publisher <x> <y> <z> <yaw_deg> [<delay>]\n");
		return -1;
	}
}

void WayPointPublisher::setDesiredPos(double x, double y, double z)
{
	desired_position << x, y, z;
}

void WayPointPublisher::setDesiredYaw(double yaw)
{
	desired_yaw = yaw;
}

void WayPointPublisher::setDesiredPosYaw(double x, double y, double z, double yaw)
{
	desired_position << x, y, z;
	desired_yaw = yaw;
}

void WayPointPublisher::setTrajFromPosYaw()
{
	mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);
}

void WayPointPublisher::setTrajFromPosYaw(double x, double y, double z, double yaw)
{
	desired_position << x, y, z;
	desired_yaw = yaw;
	mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);
}

void WayPointPublisher::waitForCreatePublisher()
{
	ros::Duration(delay).sleep();
	ROS_INFO("Waitting for creating publisher.........");
}

void WayPointPublisher::checkSubscriberOfTrajectoryPub()
{
	while (trajectory_pub.getNumSubscribers() == 0 && ros::ok()) 
	{
		ROS_INFO("There is no subscriber available, trying again in 1 second.");
		ros::Duration(1.0).sleep();
	}
}

ros::NodeHandle WayPointPublisher::getNodeHandler()
{
	return nh;
}

Eigen::Vector3d WayPointPublisher::getDesiredPos()
{
	return desired_position;
}

void WayPointPublisher::publishTraj()
{
	trajectory_pub.publish(trajectory_msg);
}

void WayPointPublisher::waitForTrajMsg()
{
	ROS_INFO("Waiting path message");

   // WaitforMessage setting
	boost::shared_ptr<trajectory_msgs::MultiDOFJointTrajectoryPoint const> sharedEdge;	
	sharedEdge = ros::topic::waitForMessage<trajectory_msgs::MultiDOFJointTrajectoryPoint>("/desired_trajectory");

	if(sharedEdge != NULL)
	{
		ROS_INFO("Getting path message");
		ros::spinOnce();
	}
}

void WayPointPublisher::setTrajFromCallBack()
{
	desired_position << d_x, d_y, d_z;
	mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);

	trajectory_msg.points[0].velocities[0].linear.x = v_x;
	trajectory_msg.points[0].velocities[0].linear.y = v_y;
	trajectory_msg.points[0].velocities[0].linear.z = v_z;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "waypoint_publisher");

	WayPointPublisher wp;

	ros::V_string args;
	ros::removeROSArgs(argc, argv, args);

	wp.setDelay(args);

	wp.setTrajFromPosYaw(std::stof(args.at(1)), std::stof(args.at(2)),std::stof(args.at(3)), -90*DEG_2_RAD);

	// Wait for some time to create the ros publisher.
	wp.waitForCreatePublisher();

	// Checking if any node subscribes trajectory_pub
	wp.checkSubscriberOfTrajectoryPub();

	ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",wp.getNodeHandler().getNamespace().c_str()
		, wp.getDesiredPos().x()
		, wp.getDesiredPos().y()
		, wp.getDesiredPos().z());

	wp.publishTraj();

	sleep(5);

	wp.waitForTrajMsg();

	while(ros::ok())
	{
		ros::spinOnce();
		wp.setTrajFromCallBack();
		wp.publishTraj();
	}

	ros::spinOnce();
	ros::shutdown();

	return 0;
}
