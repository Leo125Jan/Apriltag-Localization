#include <set>
#include <string>
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
#include <type_traits>
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

class CameraCalibration
{
public:

	CameraCalibration();
	CameraCalibration(string Type, string pubTopic, string subTopic);

	void OdeCallback(const geometry_msgs::Pose::ConstPtr &msg);
	void cameraCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);
	void Print_error(float x, float y, float z);
	Eigen::Quaterniond Quaterion_calcutaion(float i, float j, float k, float w, float x, float y, float z);

	void setTagPos(vector<int> id);
	void waitForMessage();

	void publish();
private:

	string type;
	///////////////publisher subscriber/////////////

	ros::NodeHandle nh;
	ros::Publisher Pose_detect;
	ros::Subscriber ode_sub;
	ros::Subscriber tag_sub;

	///////////////Tag position/////////////

	float tag_x;
	float tag_y;
	float tag_z;

	///////////////odometry/////////////

	float ode_x;
	float ode_y;
	float ode_z;

	///////////////camera/////////////

	float x_err;
	float y_err;
	float z_err;
	float cal_x;
	float cal_y;
	float cal_z;
	Eigen::Quaterniond q;
	std_msgs::Header header;
	apriltag_ros::AprilTagDetectionArray tag;
	float x;
	float y;
	float z;
	float i;
	float j;
	float k;
	float w;
	boost::array<float, 36> cov;
	vector<int> id;
	geometry_msgs::PoseWithCovarianceStamped Calibration;

};

CameraCalibration::CameraCalibration(string Type, string pubTopic, string subTopic)
{
	type = Type;

	Pose_detect = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(pubTopic, 1000);
	tag_sub = nh.subscribe(subTopic, 1000, &CameraCalibration::cameraCallback, this);
	ode_sub = nh.subscribe("/iris1/odometry_sensor1/pose", 1000, &CameraCalibration::OdeCallback, this);

	if(type == "downward")
		tag_x = tag_y = tag_z = 0.05;
	else if(type == "forward")
		tag_x = tag_y = tag_z = 0;
}

Eigen::Quaterniond CameraCalibration::Quaterion_calcutaion(float i, float j, float k, float w, float x, float y, float z)
{
	Eigen::Quaterniond q1;
	Eigen::Quaterniond q2;
	Eigen::Quaterniond q3;

	q1.x() = i; q1.y() = j; q1.z() = k; q1.w() = w;
	q2.x() = x; q2.y() = y; q2.z() = z; q2.w() = 0;

	q3 = q1.inverse() * q2 * q1;

	return q3;
}

void CameraCalibration::OdeCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
	ode_x = (*msg).position.x;
	ode_y = (*msg).position.y;
    ode_z = (*msg).position.z;
}

void CameraCalibration::cameraCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
	
	tag = *msg;

	id = tag.detections[0].id;
	header = tag.detections[0].pose.header;
	cov = tag.detections[0].pose.pose.covariance;
	x = tag.detections[0].pose.pose.pose.position.x;
	y = tag.detections[0].pose.pose.pose.position.y;
    z = tag.detections[0].pose.pose.pose.position.z;
    i = tag.detections[0].pose.pose.pose.orientation.x;
    j = tag.detections[0].pose.pose.pose.orientation.y;
    k = tag.detections[0].pose.pose.pose.orientation.z;
    w = tag.detections[0].pose.pose.pose.orientation.w;

    q = Quaterion_calcutaion(i, j, k, w, x, y, z);

	Print_error(-q.x(), -q.y(), -q.z());

	Calibration.header = header;
	Calibration.pose.covariance = cov;
	Calibration.header.frame_id = "iris1/odometry_sensor1";
	Calibration.pose.pose.position.x = cal_x;
	Calibration.pose.pose.position.y = cal_y;
	Calibration.pose.pose.position.z = cal_z;
}


void CameraCalibration::Print_error(float x, float y, float z)
{

	setTagPos(id);
	if(type == "downward")
	{
		cal_x = x + tag_x;
		cal_y = y + tag_y;
		cal_z = z + 0.01 + tag_z;
	}
	else if(type == "forward")
	{
		cal_x = tag_x - (z + 0.05) - 0.1;
		cal_y = tag_y + y;
		cal_z = tag_z + x - 0.02;
	}

	// cout << "calbration_d_x: " << cald_x << ", calbration_d_y: " << cald_y << ", calbration_d_z: " << cald_z << endl;

	// d_x_err = abs(ode_x - cald_x);
	// d_y_err = abs(ode_y - cald_y);
	// d_z_err = abs(ode_z - cald_z);

	// cout << "downward_error_x: " << d_x_err << ", downward_error_y: " << d_y_err 
	// 	<< ", downward_error_z: " << d_z_err << endl << endl;
}

void CameraCalibration::setTagPos(vector<int> id)
{
	if(id.size() != 0)
	{
		if(type == "downward")
		{
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
		}	
		if(type == "forward")
		{
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
		}	
	}
}

void CameraCalibration::publish()
{
	Pose_detect.publish(Calibration);
}

void waitForMessage()
{
	ROS_INFO("Wait for take off");

	boost::shared_ptr<std_msgs::Int64 const> sharedEdge;
	sharedEdge = ros::topic::waitForMessage<std_msgs::Int64>("/iris1/start_det", nh);

	sleep(1);

	if(sharedEdge != NULL)
	{
		ROS_INFO("Start Detect");
	}
}

int main(int argc, char** argv)
{
	ROS_INFO("Calibration and Calculating Error");
	ros::init(argc, argv, "Error_calculator");
	CameraCalibration c_downward("downward", "/iris1/pose_d", "/downward/tag_detections");
	CameraCalibration c_forward("forward", "/iris1/pose_f", "/forward/tag_detections");
  
	waitForMessage();

	while(ros::ok())
	{
		ros::spinOnce();
		c_forward.publish();
		c_downward.publish();
	}
}
