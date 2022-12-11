#include <set>
#include <cmath>
#include <math.h>
#include <string>
#include <vector>
#include <stdio.h>
#include <cstdlib>
#include <fstream>
#include <numeric>
#include <stdlib.h>
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
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <apriltag_ros/AprilTagDetection.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

using namespace std;

class CameraCalibration
{
	public:

		// Constructor
		CameraCalibration(string Type, string pubTopic, string subTopic);

		// Callback
		void OdeCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
		void cameraCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);

		// Operator
		void setTagPos(vector<int> id);
		void SDcalculator(vector<float> v);
		void Tag2World(float x, float y, float z);
		Eigen::Quaterniond Quaterion_calcutaion(float i, float j, float k, float w, float x, float y, float z);
		Eigen::Quaterniond Coordinate_Transform(float i, float j, float k, float w, float x, float y, float z);

	private:

		//// Publisher & Subscriber ////
		ros::NodeHandle nh;
		ros::Publisher Pose_detect;
		ros::Subscriber ode_sub;
		ros::Subscriber tag_sub;
		ros::Subscriber fus_sub;

		//// Center Position of Tag ////
		float tag_x = 0.0;
		float tag_y = 0.0;
		float tag_z = 0.0;

		float tag_offset_x = +0.045;
		float tag_offset_y = 0.0;
		float tag_offset_z = (0.03 - 0.15);

		//// Transformation Operator ////
		float cal_x = 0.0;
		float cal_y = 0.0;
		float cal_z = 0.0;

		float std_x = 0.0;
		float std_y = 0.0;
		float std_z = 0.0;

		float temp_x = 0.0;
		float temp_y = 0.0;
		float temp_z = 0.0;

		float delta_x = 0.0;
		float delta_y = 0.0;
		float delta_z = 0.0;

		float x = 0.0;
		float y = 0.0;
		float z = 0.0;
		float i = 0.0;
		float j = 0.0;
		float k = 0.0;
		float w = 0.0;

		//// Attitude From OptiTrack ////
		float ode_i = 0.0;
		float ode_j = 0.0;
		float ode_k = 0.0;
		float ode_w = 0.0;

		//// Input-Type ////
		string type;
		int count = 0;
		bool Check = true;
		bool stagnation = false;
		vector<int> id;
		vector<float> bundle_v;
		Eigen::Quaterniond q;
		boost::array<float, 36> cov;

		//// Msg ////
		std_msgs::Header header;
		nav_msgs::Odometry Calibration;
		tf::TransformBroadcaster odom_broadcaster;
		geometry_msgs::TransformStamped odom_trans;

		//// Timer ////
		ros::Time start_time = ros::Time::now();
};

CameraCalibration::CameraCalibration(string Type, string pubTopic, string subTopic)
{
	type = Type;

	Pose_detect = nh.advertise<nav_msgs::Odometry>(pubTopic, 1000);
	tag_sub = nh.subscribe(subTopic, 1000, &CameraCalibration::cameraCallback, this);
	ode_sub = nh.subscribe("/vrpn_client_node/MAV1/pose", 1000, &CameraCalibration::OdeCallback, this);
}

void CameraCalibration::OdeCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	ode_i = (*msg).pose.orientation.x;
	ode_j = (*msg).pose.orientation.y;
	ode_k = (*msg).pose.orientation.z;
	ode_w = (*msg).pose.orientation.w;
}

void CameraCalibration::cameraCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
	if ((*msg).detections.size() > 0 && Check)
	{
	    if(type == "downward")
	    {
			header = (*msg).header;
			cov = (*msg).detections[0].pose.pose.covariance;

	    	for (int c = 0; c < (*msg).detections.size(); c++)
			{
				id = (*msg).detections[c].id;
				x = (*msg).detections[c].pose.pose.pose.position.x;
				y = (*msg).detections[c].pose.pose.pose.position.y;
			    z = (*msg).detections[c].pose.pose.pose.position.z;
			    i = (*msg).detections[c].pose.pose.pose.orientation.x;
			    j = (*msg).detections[c].pose.pose.pose.orientation.y;
			    k = (*msg).detections[c].pose.pose.pose.orientation.z;
			    w = (*msg).detections[c].pose.pose.pose.orientation.w;

			    q = Quaterion_calcutaion(i, j, k, w, x, y, z);
				Tag2World(-q.x(), -q.y(), -q.z());

				bundle_v.push_back(std_x);
				bundle_v.push_back(std_y);
				bundle_v.push_back(std_z);
			}

			SDcalculator(bundle_v);
	    }

	    // Static Transformation
	    // odom_trans.header = header;
	    // odom_trans.header.frame_id = "odom";
	    // odom_trans.child_frame_id = "base_link";
	    // odom_trans.transform.translation.x = cal_x;
	    // odom_trans.transform.translation.y = cal_y;
	    // odom_trans.transform.translation.z = cal_z;
	    // odom_trans.transform.rotation.x = 0;
	    // odom_trans.transform.rotation.y = 0;
	    // odom_trans.transform.rotation.z = 0;
	    // odom_trans.transform.rotation.w = 1;

	    // odom_broadcaster.sendTransform(odom_trans);

	    // Publish coordinate
		Calibration.header = header;
		Calibration.pose.covariance = cov;
		Calibration.header.frame_id = "odom";
		Calibration.child_frame_id = "base_link";
		Calibration.pose.pose.position.x = cal_x;
		Calibration.pose.pose.position.y = cal_y;
		Calibration.pose.pose.position.z = cal_z;
		Calibration.pose.pose.orientation.x = ode_i;
		Calibration.pose.pose.orientation.y = ode_j;
		Calibration.pose.pose.orientation.z = ode_k;
		Calibration.pose.pose.orientation.w = ode_w;

		Pose_detect.publish(Calibration);
	}
	else
	{
		if (count == 0)
		{
			ROS_WARN("Not found");
			Check = false;
			count ++;

			start_time = ros::Time::now();
		}

		if(type == "downward" && (ros::Time::now() - start_time) < ros::Duration(1))
	    {	
			ROS_INFO("stagnation");
			double t_ = (ros::Time::now().toSec() - start_time.toSec());

			cal_x = 0.8*cal_x;
			cal_y = 0.8*cal_y;
			cal_z = temp_z + 0.6*(0.5*(1+tanh((4*(t_)-2))))*(1-temp_z);

			Calibration.header = (*msg).header;
			Calibration.pose.covariance = cov;
			Calibration.pose.pose.position.x = cal_x;
			Calibration.pose.pose.position.y = cal_y;
			Calibration.pose.pose.position.z = cal_z;
			Calibration.pose.pose.orientation.x = ode_i;
			Calibration.pose.pose.orientation.y = ode_j;
			Calibration.pose.pose.orientation.z = ode_k;
			Calibration.pose.pose.orientation.w = ode_w;
			Pose_detect.publish(Calibration);
		}
		else
		{
			count = 0;
			Check = true;
			start_time = ros::Time::now();
			stagnation = true;
		}
	}
}

Eigen::Quaterniond CameraCalibration::Quaterion_calcutaion(float i, float j, float k, float w, float x, float y, float z)
{
	Eigen::Quaterniond q1;
	Eigen::Quaterniond q2;

	q1.x() = i; q1.y() = j; q1.z() = k; q1.w() = w;
	q2.x() = x; q2.y() = y; q2.z() = z; q2.w() = 0;

	return q1.inverse()*q2*q1;
}

void CameraCalibration::Tag2World(float x, float y, float z)
{
	setTagPos(id);

	if(type == "downward")
	{
		std_x = x + tag_x + tag_offset_x;
		std_y = y + tag_y + tag_offset_y;

		if (z > 0)
		{
			std_z = z + tag_z + tag_offset_z;
		}
		else
		{
			std_z = -z + tag_z + tag_offset_z;
		}
	}
}

void CameraCalibration::SDcalculator(vector<float> v)
{
	// Operator
	int sequence = v.size()/3;
	float sigma = 0.0;
	float sigma_h = 0.0;
	float ari_mean = 0.0;
	float ari_mean_h = 0.0;

	for (int c = 0; c < sequence; c++)
	{
		ari_mean_h = ari_mean_h + sqrt(pow(v.at(3*c), 2) + pow(v.at(3*c+1), 2) + pow(v.at(3*c+2), 2));
	}

	ari_mean = ari_mean_h/sequence;

	for (int c = 0; c < sequence; c++)
	{
		sigma_h = sigma_h + pow((sqrt(pow(v.at(3*c), 2) + pow(v.at(3*c+1), 2) + pow(v.at(3*c+2), 2)) - ari_mean), 2);
	}

	sigma = sqrt(sigma_h/sequence);

	// Neglect outliner
	vector <bool> check_point;
	vector <float> x_hold;
	vector <float> y_hold;
	vector <float> z_hold;

	for (int c = 0; c < sequence; c++)
	{
		if (abs(sqrt(pow(v.at(3*c+0), 2) + pow(v.at(3*c+1), 2) + pow(v.at(3*c+2), 2)) - ari_mean) > 1.0*sigma)
		{
			check_point.push_back(false);
		}
		else
		{
			check_point.push_back(true);
		}
	}

	for (int c = 0; c < sequence; c++)
	{
		if (check_point.at(c))
		{
			x_hold.push_back(v.at(3*c+0));
			y_hold.push_back(v.at(3*c+1));
			z_hold.push_back(v.at(3*c+2));
		}
	}

	if (x_hold.size() > 0)
	{
		cal_x = accumulate(x_hold.begin(), x_hold.end(), 0.0f)/x_hold.size();
		cal_y = accumulate(y_hold.begin(), y_hold.end(), 0.0f)/y_hold.size();
		cal_z = accumulate(z_hold.begin(), z_hold.end(), 0.0f)/z_hold.size();
	}

	// Check for large gap between last and prev
	if (stagnation) // Pass after stagnation
	{
		if ((ros::Time::now() - start_time) < ros::Duration(1))
		{
			double t_ = (ros::Time::now().toSec() - start_time.toSec());
			cal_x = (0.5*(1+tanh((4*(t_)-2))))*cal_x;
			cal_y = (0.5*(1+tanh((4*(t_)-2))))*cal_y;
		}
		else
		{
			stagnation = false;
			temp_x = cal_x;
			temp_y = cal_y;
			temp_z = cal_z;
		}	
	}
	else
	{
		if (abs(cal_x - temp_x) > 0.3)
		{
			Check = false;
			cal_x = temp_x;
			cal_y = temp_y;
			cal_z = temp_z;
		}
		else
		{
			temp_x = cal_x;
		}

		if (abs(cal_y - temp_y) > 0.3)
		{
			Check = false;
			cal_x = temp_x;
			cal_y = temp_y;
			cal_z = temp_z;
		}
		else
		{
			temp_y = cal_y;
		}
		
		if (cal_z > -0.1)
		{
			temp_z = cal_z;
		}
		else
		{
			cal_z = temp_z;
		}
	}

	// cout << "calbration_" << type << "_x: "<< cal_x 
	// << ", calbration_" << type << "_y: " << cal_y 
	// << ", calbration_" << type << "_z: " << cal_z << "\n";

	bundle_v.clear();
	check_point.clear();
	x_hold.clear(); y_hold.clear(); z_hold.clear();
}

void CameraCalibration::setTagPos(vector<int> id)
{
	if(id.size() != 0)
	{
		if(type == "downward")
		{
			switch (id[0])
			{
				case 40: //RB

					tag_x = 0.42768155;
					tag_y = -0.48527310;

					break;

				case 56: //CB

					tag_x = -0.07486597;
					tag_y = -0.47853970;

					break;

				case 60: //LB

					tag_x = -0.42448583;
					tag_y = -0.47745534;

					break;

				case 68: //LC

					tag_x = -0.41786661;
					tag_y = 0.06825715;

					break;

				case 76: //LT

					tag_x = -0.41847062;
					tag_y = 0.67012691;

					break;

				case 84: //CT

					tag_x = 0.07901816;
					tag_y = 0.67185808;

					break;

				case 88: //RT

					tag_x = 0.42635642;
					tag_y = 0.66976314;

					break;

				case 28: //RC

					tag_x = 0.42945725;
					tag_y = 0.07208341;

					break;

				case 1: //CC

					tag_x = 0;
					tag_y = 0;

					break;

				default:

					tag_x = 0.0;
					tag_y = 0.0;

					break;
			}
		}
	}
}

int main(int argc, char** argv)
{
	ROS_INFO("Start Coordinate Transformstion");
	ros::init(argc, argv, "Error_calculator");

	CameraCalibration c_downward("downward", "/camera/pose_d", "/downward/tag_detections");

	ros::Rate rate(100);

	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}
}