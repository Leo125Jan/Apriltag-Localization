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

class ErrorCalculate
{
public:

	ErrorCalculate();

	void OdeCallback(const geometry_msgs::Pose::ConstPtr &msg);
	void downCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);
	void forCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);
	void Print_downward_error(float x, float y, float z, vector<int> id);
	void Print_forward_error(float x, float y, float z, vector<int> id);
	Eigen::Quaterniond Quaterion_calcutaion(float i, float j, float k, float w, float x, float y, float z);

	void setForwardTagPos(vector<int> id);
	void setDownwardTagPos(vector<int> id);

	void waitForMessage();

	void publish();
private:

	///////////////publisher subscriber/////////////

	ros::NodeHandle nh;
	ros::Publisher Pose_detect_forward;
	ros::Publisher Pose_detect_downward;
	ros::Subscriber ode_sub;
	ros::Subscriber downward_tag;
	ros::Subscriber forward_tag;

	///////////////Tag position/////////////

	float tagD_x;
	float tagD_y;
	float tagD_z;
	float tagF_x;
	float tagF_y;
	float tagF_z;

	///////////////odometry/////////////

	float ode_x;
	float ode_y;
	float ode_z;

	///////////////camera_downward/////////////

	float d_x_err;
	float d_y_err;
	float d_z_err;
	float cald_x;
	float cald_y;
	float cald_z;
	Eigen::Quaterniond q_d;
	std_msgs::Header header_d;
	apriltag_ros::AprilTagDetectionArray down_b;
	float x_b;
	float y_b;
	float z_b;
	float i_b;
	float j_b;
	float k_b;
	float w_b;
	boost::array<float, 36> cov_d;
	vector<int> id_d;
	geometry_msgs::PoseWithCovarianceStamped Calibration_d;

	///////////////camera_forward/////////////

	float f_x_err;
	float f_y_err;
	float f_z_err;
	float calf_x;
	float calf_y;
	float calf_z;
	Eigen::Quaterniond q_f;
	std_msgs::Header header_f;
	apriltag_ros::AprilTagDetectionArray down_f;
	float x_f;
	float y_f;
	float z_f;
	float i_f;
	float j_f;
	float k_f;
	float w_f;
	boost::array<float, 36> cov_f;
	vector<int> id_f;
	geometry_msgs::PoseWithCovarianceStamped Calibration_f;

};

ErrorCalculate::ErrorCalculate()
{
	ROS_INFO("Calibration and Calculating Error");

	Pose_detect_downward = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/iris1/pose_d", 1000);
	Pose_detect_forward = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/iris1/pose_f", 1000);
	
	forward_tag = nh.subscribe("/forward/tag_detections", 1000, &ErrorCalculate::forCallback, this);
	downward_tag = nh.subscribe("/downward/tag_detections", 1000, &ErrorCalculate::downCallback, this);
	ode_sub = nh.subscribe("/iris1/odometry_sensor1/pose", 1000, &ErrorCalculate::OdeCallback, this);

	tagD_x = tagD_y = tagD_z = 0.05;
	tagF_x = tagF_y = tagF_z = 0;
}

void ErrorCalculate::waitForMessage()
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

Eigen::Quaterniond ErrorCalculate::Quaterion_calcutaion(float i, float j, float k, float w, float x, float y, float z)
{
	Eigen::Quaterniond q1;
	Eigen::Quaterniond q2;
	Eigen::Quaterniond q3;

	q1.x() = i; q1.y() = j; q1.z() = k; q1.w() = w;
	q2.x() = x; q2.y() = y; q2.z() = z; q2.w() = 0;

	q3 = q1.inverse() * q2 * q1;

	return q3;
}

void ErrorCalculate::OdeCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
	ode_x = (*msg).position.x;
	ode_y = (*msg).position.y;
    ode_z = (*msg).position.z;
}

void ErrorCalculate::downCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
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

	Calibration_d.header = header_d;
	Calibration_d.pose.covariance = cov_f;
	Calibration_d.header.frame_id = "iris1/odometry_sensor1";
	Calibration_d.pose.pose.position.x = cald_x;
	Calibration_d.pose.pose.position.y = cald_y;
	Calibration_d.pose.pose.position.z = cald_z;
}

void ErrorCalculate::forCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
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

    q_f = Quaterion_calcutaion(i_f, j_f, k_f, w_f, x_f, y_f, z_f);

    Print_forward_error(-q_f.x(), -q_f.y(), -q_f.z(), id_f);

    Calibration_f.header = header_f;
	Calibration_f.pose.covariance = cov_f;
	Calibration_f.header.frame_id = "iris1/odometry_sensor1";
	Calibration_f.pose.pose.position.x = calf_x;
	Calibration_f.pose.pose.position.y = calf_y;
	Calibration_f.pose.pose.position.z = calf_z;
}

void ErrorCalculate::Print_downward_error(float x, float y, float z, vector<int> id)
{
	setDownwardTagPos(id);
	cald_x = x + tagD_x;
	cald_y = y + tagD_y;
	cald_z = z + 0.01 + tagD_z;

	// cout << "calbration_d_x: " << cald_x << ", calbration_d_y: " << cald_y << ", calbration_d_z: " << cald_z << endl;

	// d_x_err = abs(ode_x - cald_x);
	// d_y_err = abs(ode_y - cald_y);
	// d_z_err = abs(ode_z - cald_z);

	// cout << "downward_error_x: " << d_x_err << ", downward_error_y: " << d_y_err 
	// 	<< ", downward_error_z: " << d_z_err << endl << endl;
}

void ErrorCalculate::Print_forward_error(float x, float y, float z, vector<int> id)
{
	setForwardTagPos(id);
	calf_x = tagF_x - (z + 0.05) - 0.1;
	calf_y = tagF_y + y;
	calf_z = tagF_z + x - 0.02;

	// cout << "calbration_f_x: " << calf_x << ", calbration_f_y: " << calf_y << ", calbration_f_z: " << calf_z << endl;

	// f_x_err = abs(ode_x - calf_x);
	// f_y_err = abs(ode_y - calf_y);
	// f_z_err = abs(ode_z - calf_z);

	// cout << "forward_error_x: " << f_x_err << ", forward_error_y: " << f_y_err 
	// 	<< ", forward_error_z: " << f_z_err << endl << endl;
}

void ErrorCalculate::setDownwardTagPos(vector<int> id)
{
	if(id.size() != 0)
		switch (id[0])
		{
			case 0:

				tagD_x = -6.85;
				tagD_y = 0;

				break;

			case 6:

				tagD_x = -6.0;
				tagD_y = 0;

				break;

			case 7:

				tagD_x = -5.0;
				tagD_y = 0;

				break;

			case 8:

				tagD_x = -4.0;
				tagD_y = 0;

				break;

			case 9:

				tagD_x = -3.0;
				tagD_y = 0;

				break;

			case 10:

				tagD_x = -2.0;
				tagD_y = 0;

				break;

			case 11:

				tagD_x = -1.0;
				tagD_y = 0;

				break;

			case 12:

				tagD_x = 0.0;
				tagD_y = 0;

				break;

			case 13:

				tagD_x = 1.0;
				tagD_y = 0;

				break;

			case 14:

				tagD_x = 2.0;
				tagD_y = 0;

				break;

			case 15:

				tagD_x = 3.0;
				tagD_y = 0;

				break;

			case 16:

				tagD_x = 4.0;
				tagD_y = 0;

				break;

			case 17:

				tagD_x = 5.0;
				tagD_y = 0;

				break;

			case 18:

				tagD_x = 6.0;
				tagD_y = 0;

				break;

			default:

				tagD_x = -7;
				tagD_y = 0;

				break;
		}
	
}

void ErrorCalculate::setForwardTagPos(vector<int> id)
{
	if(id.size() != 0)
		switch (id[0])
		{
			case 1:

				tagF_x = 8.0;
				tagF_y = 0;
				tagF_z = 2;

				break;

			default:

				tagF_x = -7;
				tagF_y = 0;

				break;
		}	
}

void ErrorCalculate::publish()
{
	Pose_detect_downward.publish(Calibration_d);
	Pose_detect_forward.publish(Calibration_f);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Error_calculator");
	ErrorCalculate ec;

	//ec.waitForMessage();

	while(ros::ok())
	{
		ros::spinOnce();
		ec.publish();
	}
}
