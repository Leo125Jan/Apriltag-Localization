#include <set>
#include <cmath>
#include <vector>
#include <stdio.h>
#include <cstdlib>
#include <fstream>
#include <unistd.h>
#include <iostream>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <boost/array.hpp>
#include <nav_msgs/Path.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Header.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <apriltag_ros/AprilTagDetection.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

using namespace std;
using namespace Eigen;

const float DEG_2_RAD = M_PI / 180.0;

geometry_msgs::Pose self_ode;

float ode_x, ode_y, ode_z;

void OdeCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
	self_ode = *msg;

	ode_x = self_ode.position.x;
	ode_y = self_ode.position.y;
    ode_z = self_ode.position.z;
}

float d_x_err, d_y_err, d_z_err;
float cald_x, cald_y, cald_z;

void Print_downward_error(float x, float y, float z, vector<int> id)
{
	float tag_x, tag_y, tag_z = 0.05;

	switch (id[0])
	{
		case 0:

			tag_x = -6.85;
			tag_y = 0;

			break;

		case 6:

			tag_x = -6.0;
			tag_y = 0;

			break;

		case 7:

			tag_x = -5.0;
			tag_y = 0;

			break;

		case 8:

			tag_x = -4.0;
			tag_y = 0;

			break;

		case 9:

			tag_x = -3.0;
			tag_y = 0;

			break;

		case 10:

			tag_x = -2.0;
			tag_y = 0;

			break;

		case 11:

			tag_x = -1.0;
			tag_y = 0;

			break;

		case 12:

			tag_x = 0.0;
			tag_y = 0;

			break;

		case 13:

			tag_x = 1.0;
			tag_y = 0;

			break;

		case 14:

			tag_x = 2.0;
			tag_y = 0;

			break;

		case 15:

			tag_x = 3.0;
			tag_y = 0;

			break;

		case 16:

			tag_x = 4.0;
			tag_y = 0;

			break;

		case 17:

			tag_x = 5.0;
			tag_y = 0;

			break;

		case 18:

			tag_x = 6.0;
			tag_y = 0;

			break;

		default:

			tag_x = -7;
			tag_y = 0;

			break;
	}

	cald_x = x + tag_x;
	cald_y = y + tag_y;
	cald_z = z + 0.01 + tag_z;

	cout << "calbration_d_x: " << cald_x << ", calbration_d_y: " << cald_y << ", calbration_d_z: " << cald_z << endl;

	d_x_err = abs(ode_x - cald_x);
	d_y_err = abs(ode_y - cald_y);
	d_z_err = abs(ode_z - cald_z);

	cout << "downward_error_x: " << d_x_err << ", downward_error_y: " << d_y_err 
		<< ", downward_error_z: " << d_z_err << endl << endl;
}

float f_x_err, f_y_err, f_z_err;
float calf_x, calf_y, calf_z;

void Print_forward_error(float x, float y, float z, vector<int> id)
{
	float tag_x, tag_y, tag_z;

	switch (id[0])
	{
		case 1:

			tag_x = 8.0;
			tag_y = 0;
			tag_z = 2;

			break;

		default:

			tag_x = -7;
			tag_y = 0;

			break;
	}	
	calf_x = tag_x - (z + 0.05) - 0.1;
	calf_y = tag_y + y;
	calf_z = tag_z + x - 0.02;

	// cout << "calbration_f_x: " << calf_x << ", calbration_f_y: " << calf_y << ", calbration_f_z: " << calf_z << endl;

	// f_x_err = abs(ode_x - calf_x);
	// f_y_err = abs(ode_y - calf_y);
	// f_z_err = abs(ode_z - calf_z);

	// cout << "forward_error_x: " << f_x_err << ", forward_error_y: " << f_y_err 
	// 	<< ", forward_error_z: " << f_z_err << endl << endl;
}

Eigen::Quaterniond Quaterion_calcutaion(float i, float j, float k, float w, float x, float y, float z)
{
	Eigen::Quaterniond q1;
	Eigen::Quaterniond q2;
	Eigen::Quaterniond q3;

	q1.x() = i; q1.y() = j; q1.z() = k; q1.w() = w;
	q2.x() = x; q2.y() = y; q2.z() = z; q2.w() = 0;

	q3 = q1.inverse() * q2 * q1;

	return q3;
}

Eigen::Quaterniond q_d;
std_msgs::Header header_d;
apriltag_ros::AprilTagDetectionArray down_b;

float x_b, y_b, z_b, i_b, j_b, k_b, w_b;
boost::array<float, 36> cov_d;
vector<int> id_d;

void downCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
	if (msg->detections.size() == 0)
	{
		ROS_INFO("No tag detected");
	}
	else
	{
		down_b = *msg;

		id_d = down_b.detections[0].id;
		header_d = down_b.detections[0].pose.header;
		cov_d = down_b.detections[0].pose.pose.covariance;
		x_b = down_b.detections[0].pose.pose.pose.position.x;
		y_b = down_b.detections[0].pose.pose.pose.position.y;
	    z_b = down_b.detections[0].pose.pose.pose.position.z;
	    i_b = down_b.detections[0].pose.pose.pose.orientation.x;
	    j_b = down_b.detections[0].pose.pose.pose.orientation.y;
	    k_b = down_b.detections[0].pose.pose.pose.orientation.z;
	    w_b = down_b.detections[0].pose.pose.pose.orientation.w;

	    q_d = Quaterion_calcutaion(i_b, j_b, k_b, w_b, x_b, y_b, z_b);

		Print_downward_error(-q_d.x(), -q_d.y(), -q_d.z(), id_d);
	}
}

Eigen::Quaterniond q_f;
std_msgs::Header header_f;
apriltag_ros::AprilTagDetectionArray down_f;

float x_f, y_f, z_f, i_f, j_f, k_f, w_f;
boost::array<float, 36> cov_f;
vector<int> id_f;

void forCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
	down_f = *msg;

	id_f = down_f.detections[0].id;
	header_f = down_f.detections[0].pose.header;
	cov_f = down_f.detections[0].pose.pose.covariance;
	x_f = down_f.detections[0].pose.pose.pose.position.x;
	y_f = down_f.detections[0].pose.pose.pose.position.y;
    z_f = down_f.detections[0].pose.pose.pose.position.z;
    i_f = down_b.detections[0].pose.pose.pose.orientation.x;
    j_f = down_b.detections[0].pose.pose.pose.orientation.y;
    k_f = down_b.detections[0].pose.pose.pose.orientation.z;
    w_f = down_b.detections[0].pose.pose.pose.orientation.w;

    // q_f = Quaterion_calcutaion(i_f, j_f, k_f, w_f, x_f, y_f, z_f);

    // Print_forward_error(-q_f.x(), -q_f.y(), -q_f.z(), id_f);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Error_calculator");
	ros::NodeHandle nh;

	ROS_INFO("Calibration and Calculating Error");

	// Node setting
	ros::Publisher Pose_detect_forward;
	ros::Publisher Pose_detect_downward;
	ros::Subscriber ode_sub;
	ros::Subscriber downward_tag;
	ros::Subscriber forward_tag;

	// Variable setting
	Pose_detect_downward = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/iris1/pose_d", 1000);
	Pose_detect_forward = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/iris1/pose_f", 1000);

	forward_tag = nh.subscribe("/forward/tag_detections", 1000, forCallback);
	downward_tag = nh.subscribe("/downward/tag_detections", 1000, downCallback);
	ode_sub = nh.subscribe("/iris1/odometry_sensor1/pose", 1000, OdeCallback);

	geometry_msgs::PoseWithCovarianceStamped Calibration_d;
	geometry_msgs::PoseWithCovarianceStamped Calibration_f;

	ROS_INFO("Wait for take off");

	// Wait for message
	// boost::shared_ptr<std_msgs::Int64 const> sharedEdge;
	// sharedEdge = ros::topic::waitForMessage<std_msgs::Int64>("/iris1/start_det", nh);

	// sleep(1);

	// if(sharedEdge != NULL)
	// {
	// 	ROS_INFO("Start Detect");
	// }

	while (ros::ok())
	{
		ros::spinOnce();

		Calibration_d.header = header_d;
		Calibration_d.pose.covariance = cov_f;
		Calibration_d.header.frame_id = "iris1/odometry_sensor1";
		Calibration_d.pose.pose.position.x = cald_x;
		Calibration_d.pose.pose.position.y = cald_y;
		Calibration_d.pose.pose.position.z = cald_z;

		Calibration_f.header = header_f;
		Calibration_d.pose.covariance = cov_f;
		Calibration_f.header.frame_id = "iris1/odometry_sensor1";
		Calibration_f.pose.pose.position.x = calf_x;
		Calibration_f.pose.pose.position.y = calf_y;
		Calibration_f.pose.pose.position.z = calf_z;

		Pose_detect_downward.publish(Calibration_d);
		Pose_detect_forward.publish(Calibration_f);
	}
	
	return 0;
}