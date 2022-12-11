#include <set>
#include <cmath>
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
using namespace Eigen;
const float DEG_2_RAD = M_PI / 180.0;

class CameraCalibration
{
	public:

		CameraCalibration();
		CameraCalibration(string Type, string pubTopic, string subTopic);

		void Print_error(float x, float y, float z);
		void OdeCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
		void cameraCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);
		Eigen::Quaterniond Quaterion_calcutaion(float i, float j, float k, float w, float x, float y, float z);
		Eigen::Quaterniond Coordinate_Transform(float i, float j, float k, float w, float x, float y, float z);

		void publish();
		void setTagPos(vector<int> id);

		void SDcalculator(vector<float> v);
		void Calibration_Callback(const nav_msgs::Odometry::ConstPtr &msg);

	private:

		string A;
		string type;
		int iteration;
		///////////////publisher subscriber/////////////
		ros::NodeHandle nh;
		ros::Publisher Pose_detect;
		ros::Subscriber ode_sub;
		ros::Subscriber tag_sub;
		ros::Subscriber fus_sub;

		///////////////Tag position/////////////
		float tag_x = 0.0;
		float tag_y = 0.0;
		float tag_z = 0.0;

		///////////////camera/////////////
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
		float ode_i = 0.0;
		float ode_j = 0.0;
		float ode_k = 0.0;
		float ode_w = 0.0;

		float fus_x = 0.08;
		float fus_y = 0.0;
		float fus_z = 0.4;

		///////////////msg/////////////
		bool Check;
		int count = 0;
		vector<int> id;
		Eigen::Quaterniond q;
		vector<float> bundle_v;
		std_msgs::Header header;
		boost::array<float, 36> cov;
		nav_msgs::Odometry Calibration;
		apriltag_ros::AprilTagDetectionArray tag;
		tf::TransformBroadcaster odom_broadcaster;
		geometry_msgs::TransformStamped odom_trans;
};

CameraCalibration::CameraCalibration(string Type, string pubTopic, string subTopic)
{
	type = Type;

	Pose_detect = nh.advertise<nav_msgs::Odometry>(pubTopic, 1000);
	tag_sub = nh.subscribe(subTopic, 1000, &CameraCalibration::cameraCallback, this);
	fus_sub = nh.subscribe("/odometry/filtered", 100, &CameraCalibration::Calibration_Callback, this);
	ode_sub = nh.subscribe("/vrpn_client_node/MAV1/pose", 1000, &CameraCalibration::OdeCallback, this);

	if(type == "downward")
	{
		tag_x = tag_y = tag_z = 0.0;
	}
	iteration = 0;
}

Eigen::Quaterniond CameraCalibration::Quaterion_calcutaion(float i, float j, float k, float w, float x, float y, float z)
{
	Eigen::Quaterniond q1;
	Eigen::Quaterniond q2;

	q1.x() = i; q1.y() = j; q1.z() = k; q1.w() = w;
	q2.x() = x; q2.y() = y; q2.z() = z; q2.w() = 0;

	return q1.inverse()*q2*q1;
}

void CameraCalibration::OdeCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	// x = (*msg).pose.position.x;
	// y = (*msg).pose.position.y;
	// z = (*msg).pose.position.z;
	ode_i = (*msg).pose.orientation.x;
	ode_j = (*msg).pose.orientation.y;
  ode_k = (*msg).pose.orientation.z;
  ode_w = (*msg).pose.orientation.w;
}

void CameraCalibration::SDcalculator(vector<float> v)
{
	// Calculate SD
	int sequence = v.size()/3;
	float ari_mean = 0.0;
	float hold = 0.0;
	float sigma = 0.0;

	if (v.size() == 0)
	{
		cal_x = 0;
		cal_y = 0;
		cal_z = std_z;

		Calibration.header = header;
		Calibration.pose.covariance = cov;
		Calibration.header.frame_id = "odom";
		Calibration.child_frame_id = "camera_link";
		Calibration.pose.pose.position.x = cal_x;
		Calibration.pose.pose.position.y = cal_y;
		Calibration.pose.pose.position.z = cal_z;
		Calibration.pose.pose.orientation.x = ode_i;
		Calibration.pose.pose.orientation.y = ode_j;
		Calibration.pose.pose.orientation.z = ode_k;
		Calibration.pose.pose.orientation.w = ode_w;

		ros::Time start_time = ros::Time::now();
	  ros::Duration timeout(0.1); // Timeout of 0.5 seconds

	  Pose_detect.publish(Calibration);

	  
	  while(ros::Time::now() - start_time < timeout)
	  {
	  	Calibration.header = header;
	  	ROS_INFO("In");
	    Pose_detect.publish(Calibration);
	    ros::spinOnce();
    }
	}
	else
	{
		for (int i = 0; i < sequence; i++)
		{
			ari_mean = ari_mean + sqrt(pow(v.at(3*i), 2) + pow(v.at(3*i+1), 2) + pow(v.at(3*i+2), 2));
		}

		for (int i = 0; i < sequence; i++)
		{
			hold = hold + pow((sqrt(pow(v.at(3*i), 2) + pow(v.at(3*i+1), 2) + pow(v.at(3*i+2), 2)) - ari_mean), 2);
		}

		sigma = sqrt(hold/sequence);

		// Neglect outliner
		vector <bool> check_point;
		vector <float> x_hold;
		vector <float> y_hold;
		vector <float> z_hold;

		for (int i = 0; i < sequence; i++)
		{
			if (abs(sqrt(pow(v.at(3*i+0), 2) + pow(v.at(3*i+1), 2) + pow(v.at(3*i+2), 2)) - ari_mean) > sigma)
			{
				check_point.push_back(false);
			}
			else
			{
				check_point.push_back(true);
			}
		}

		for (int i = 0; i < sequence; i++)
		{
			if (check_point.at(i))
			{
				x_hold.push_back(v.at(3*i+0));
				y_hold.push_back(v.at(3*i+1));
				z_hold.push_back(v.at(3*i+2));
			}
		}

		if (x_hold.size() > 0)
		{
			cal_x = accumulate(x_hold.begin(), x_hold.end(), 0.0f)/x_hold.size();
			cal_y = accumulate(y_hold.begin(), y_hold.end(), 0.0f)/y_hold.size();
			cal_z = accumulate(z_hold.begin(), z_hold.end(), 0.0f)/z_hold.size();
		}

		cout << "calbration_" << type << "_x: "<< cal_x 
		<< ", calbration_" << type << "_y: " << cal_y 
		<< ", calbration_" << type << "_z: " << cal_z << "\n";

		bundle_v.clear();
		x_hold.clear(); y_hold.clear(); z_hold.clear();
		check_point.clear();
	}
}

void CameraCalibration::Calibration_Callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  temp_x = (*msg).pose.pose.position.x;
  temp_y = (*msg).pose.pose.position.y;
  temp_z = (*msg).pose.pose.position.z;
}

void CameraCalibration::cameraCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
	if (msg->detections.size() > 0)
	{
		Check = true;

		tag = *msg;

    if(type == "downward")
    {
		header = tag.detections[0].pose.header;
		cov = tag.detections[0].pose.pose.covariance;
		int cnt;

    	for (int c = 0; c < tag.detections.size(); c++)
		{
			id = tag.detections[c].id;
			x = tag.detections[c].pose.pose.pose.position.x;
			y = tag.detections[c].pose.pose.pose.position.y;
	    z = tag.detections[c].pose.pose.pose.position.z;
	    i = tag.detections[c].pose.pose.pose.orientation.x;
	    j = tag.detections[c].pose.pose.pose.orientation.y;
	    k = tag.detections[c].pose.pose.pose.orientation.z;
	    w = tag.detections[c].pose.pose.pose.orientation.w;

	    q = Quaterion_calcutaion(i, j, k, w, x, y, z);
			Print_error(-q.x(), -q.y(), -q.z());

			if (std_x == 0 || std_y == 0)
			{
			}
			else
			{
				bundle_v.push_back(std_x);
				bundle_v.push_back(std_y);
				bundle_v.push_back(std_z);
			}
		}

		SDcalculator(bundle_v);
    }

    // odom_trans.header = header;
    // odom_trans.header.frame_id = "odom";
    // odom_trans.child_frame_id = "camera_link";
    // odom_trans.transform.translation.x = x;
    // odom_trans.transform.translation.y = y;
    // odom_trans.transform.translation.z = 0.0;
    // odom_trans.transform.rotation.x = 0;
    // odom_trans.transform.rotation.y = 0;
    // odom_trans.transform.rotation.z = 0;
    // odom_trans.transform.rotation.w = 1;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

		Calibration.header = header;
		Calibration.pose.covariance = cov;
		Calibration.header.frame_id = "odom";
		Calibration.child_frame_id = "camera_link";
		Calibration.pose.pose.position.x = cal_x;
		Calibration.pose.pose.position.y = cal_y;
		Calibration.pose.pose.position.z = cal_z;
		Calibration.pose.pose.orientation.x = ode_i;
		Calibration.pose.pose.orientation.y = ode_j;
		Calibration.pose.pose.orientation.z = ode_k;
		Calibration.pose.pose.orientation.w = ode_w;
	}
	else
	{
		Check = false;
	}
}

void CameraCalibration::Print_error(float x, float y, float z)
{
	setTagPos(id);

	if(type == "downward")
	{
		
		if (abs(x - temp_x) > 0.3 || id.size() == 0)
		{
			std_x = 0;
		}
		else
		{
			std_x = x + tag_x + 0.045;
		}

		if (abs(y - temp_y) > 0.3 || id.size() == 0)
		{
			std_y = 0;
		}
		else
		{
			std_y = y + tag_y;
		}

		if (z > 0)
		{
			std_z = z + 0.03 + tag_z - 0.15;
		}
		else
		{
			std_z = -z + 0.03 + tag_z - 0.15;
		}
	}
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

void CameraCalibration::publish()
{
	if (Check == true)
	{
		Pose_detect.publish(Calibration);
	}
	else if (Check == false)
	{
	}
}

int main(int argc, char** argv)
{
	ROS_INFO("Calibration and Calculating Error");
	ros::init(argc, argv, "Error_calculator");

	CameraCalibration c_downward("downward", "/camera/pose_d", "/downward/tag_detections");

	ros::Rate rate(100);

	while(ros::ok())
	{
		ros::spinOnce();

		c_downward.publish();

		rate.sleep();
	}
}

