#include <cmath>
#include <fstream>
#include <unistd.h>
#include <iostream>
#include <ros/ros.h>
#include <Eigen/Core>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

using namespace std;

geometry_msgs::Pose move_ode;

float ode_x_m, ode_y_m, ode_z_m;
float C = 1;

void Ode_m_Callback(const geometry_msgs::Pose::ConstPtr &msg_m)
{
	C = C + 1;

	if (C == 3)
	{
		move_ode = *msg_m;

		ode_x_m = move_ode.position.x;
		ode_y_m = move_ode.position.y;
	    ode_z_m= move_ode.position.z;
	}
}

apriltag_ros::AprilTagDetectionArray Husky_pose;

float x, y, z, Real_x, Real_y, Real_z;
float cur_x, cur_y, cur_z;
float des_z = 0.5;

void LandCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
	Husky_pose = *msg;

	cur_x = Husky_pose.detections[0].pose.pose.pose.position.x;
	cur_y = Husky_pose.detections[0].pose.pose.pose.position.y;
    cur_z = Husky_pose.detections[0].pose.pose.pose.position.z;

    cout << "cur_x: " << cur_x << ",cur_y: " << cur_y << ",cur_z: " << cur_z << endl;
}

geometry_msgs::Pose self_ode;

float ode_x, ode_y, ode_z;

void OdeCallback(const geometry_msgs::Pose::ConstPtr &msg1)
{
	self_ode = *msg1;

	ode_x = self_ode.position.x;
	ode_y = self_ode.position.y;
    ode_z = self_ode.position.z;
}

float Standard(void)
{
	float st_x, st_y, S;

	S = sqrt(pow(cur_x,2) + pow(cur_y,2));
	st_x = cur_x/S; st_y = cur_y/S;

	return st_x, st_y;
}

void Move_to_Husky(ros::NodeHandle &nh, ros::Publisher &trajectory_pub, const float DEG_2_RAD, double desired_yaw
					, float &f_x, float &f_y)
{
	// Move to husky
	ROS_INFO("Waitting for Husky position message");

	// Node setting
	ros::Subscriber ode_sub_m;

	// Variable setting
	Eigen::Vector3d desired_position;
	apriltag_ros::AprilTagDetectionArray Husky_pose;
	trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
	ode_sub_m = nh.subscribe("/firefly/odometry_sensor1/pose", 1000, Ode_m_Callback);

	float x_h, y_h, z_h, Real_x_h, Real_y_h, Real_z_h;

	// WaitforMessage setting
	boost::shared_ptr<apriltag_ros::AprilTagDetectionArray const> sharedEdge;
	sharedEdge = ros::topic::waitForMessage<apriltag_ros::AprilTagDetectionArray>("/tag_detections");
	
	if(sharedEdge != NULL)
	{
		ROS_INFO("Getting Husky position message");
		Husky_pose = *sharedEdge;
	}

	for(int i = 1;i <= 10;i++)
	{
		ros::spinOnce();

		if (C == 3)
		{
			break;
		}
	}
	
	cout << "ode_x: " << ode_x_m << ", ode_y: " << ode_y_m << endl;

	x_h = Husky_pose.detections[0].pose.pose.pose.position.x;
	y_h = Husky_pose.detections[0].pose.pose.pose.position.y;
	z_h = Husky_pose.detections[0].pose.pose.pose.position.z;

	Real_x_h = -(y_h + 0.5) + ode_x_m;
	Real_y_h = -(x_h + 2.3) + ode_y_m;
	Real_z_h = 10;

	desired_position << Real_x_h, Real_y_h, Real_z_h;

	mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);

	trajectory_pub.publish(trajectory_msg);

	f_x = Real_x_h; f_y = Real_y_h;
}

void Landing(ros::NodeHandle &nh, ros::Publisher &trajectory_pub, const float DEG_2_RAD, double desired_yaw
			, float f_x, float f_y)
{
	//Landing
	ROS_INFO("Prepare for Landing");

	// Node setting
	ros::Publisher motor_publisher;
	ros::Subscriber ode_sub;
	ros::Subscriber land_sub;
	
	// Variable setting
	mav_msgs::Actuators actuator_msg;
	Eigen::Vector3d desired_position;
	trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
	
	land_sub = nh.subscribe("/tag_detections", 1000, LandCallback);
    ode_sub = nh.subscribe("/firefly/odometry_sensor1/pose", 1000, OdeCallback);
	motor_publisher = nh.advertise<mav_msgs::Actuators>("/firefly/command/motor_speed", 10);
	
    // WaitforMessage setting
    boost::shared_ptr<apriltag_ros::AprilTagDetectionArray const> sharedEdge;
    sharedEdge = ros::topic::waitForMessage<apriltag_ros::AprilTagDetectionArray>("/tag_detections");

    if(sharedEdge != NULL)
	{
		ROS_INFO("Getting Husky position message");
		ros::spinOnce();
	}

    while (ros::ok())
	{
		ros::spinOnce();

		if (cur_z < 3)
	    {
	    	float k_x, k_y;

	    	Real_z = cur_z;

		    for (int i = 0;i < 10;i++)
		    {
		    	cout << "3" << endl;
		    	ros::spinOnce();

		    	cur_x, cur_y = Standard();

		    	if (abs(cur_x) > 0.2)
		    	{
		    		k_x = -0.2;
		    	}
		    	else if (abs(cur_x) > 0.1)
		    	{
		    		k_x = -0.15;
		    	}
		    	else
		    	{
		    		k_x = -0.05;
		    	}

		    	if (abs(cur_y) > 0.3)
		    	{
		    		k_y = -0.2;
		    	}
		    	else if (abs(cur_y) > 0.05)
		    	{
		    		k_y = -0.17;
		    	}
		    	else
		    	{
		    		k_y = -0.05;
		    	}
		    	
		    	Real_x = k_x*(cur_y) + ode_x;
		    	Real_y = k_y*(cur_x) + ode_y;
		    	
		    	desired_position << Real_x, Real_y, Real_z;

		    	mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);

		    	trajectory_pub.publish(trajectory_msg);

		    	sleep(2);
		    }

	    	cout << "1" << endl;

	        while (cur_z > 0.2)
	        {
				Real_x = -0.1*(cur_y) + ode_x;
	        	Real_y = -0.1*(cur_x)  + ode_y;
		    	Real_z = cur_z;

		    	desired_position << Real_x, Real_y, Real_z;

		    	mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);

		    	trajectory_pub.publish(trajectory_msg);

		    	cur_z = cur_z - 0.2;
		    	sleep(1);
	        }

	        ROS_INFO("Complete Landing");

	        for (int i = 0; i < 6; i++)
	        {
	        	actuator_msg.angular_velocities.push_back(0);
	        }

	        ROS_INFO("Motors stop");

	        system("rosnode kill /firefly/lee_position_controller_node");

	        motor_publisher.publish(actuator_msg);

	        break;
	    }
	    else
	    {
	    	cout << "2" << endl;

	    	// Translation
	    	cur_x, cur_y = Standard(); 

	    	Real_x = -0.3*(cur_y) + f_x;
	        Real_y = -0.2*(cur_x) + f_y;
	    	Real_z = cur_z;

	    	cout << "Real_x: " << Real_x << ",Real_y: " << Real_y << ",Real_z: " << Real_z << endl;

	        desired_position << Real_x, Real_y, Real_z;

	        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);

	        trajectory_pub.publish(trajectory_msg);

	        // Down
	    	Real_z = cur_z - 2;

	    	cout << "Real_x_1: " << Real_x << ",Real_y_1: " << Real_y << ",Real_z_1: " << Real_z << endl;

	        desired_position << Real_x, Real_y, Real_z;

	        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);

	        trajectory_pub.publish(trajectory_msg);
	    }

	    ros::spinOnce();
	    sleep(0.5);
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

	float f_x, f_y;

	Move_to_Husky(nh, trajectory_pub, DEG_2_RAD, desired_yaw, f_x, f_y);

	sleep(5);

	Landing(nh, trajectory_pub, DEG_2_RAD, desired_yaw, f_x, f_y);

	return 0;
}


/*
//Landing
ROS_INFO("Prepare for Landing");

ros::Subscriber sub = nh.subscribe("/tag_detections", 1000, LandCallback);
ros::Subscriber sub1 = nh.subscribe("/error", 1000, LandVCallback);
mav_msgs::Actuators actuator_msg;
ros::Publisher motor_publisher;

motor_publisher = nh.advertise<mav_msgs::Actuators>("/firefly/command/motor_speed", 10);
boost::shared_ptr<apriltag_ros::AprilTagDetectionArray const> sharedEdge1;

sharedEdge1 = ros::topic::waitForMessage<apriltag_ros::AprilTagDetectionArray>("/tag_detections");

if(sharedEdge1 != NULL)
{
	ROS_INFO("Getting Husky position message");
// 	// Husky_pose = *sharedEdge;
	ros::spinOnce();
}

while (ros::ok())
{

	ros::spinOnce();

    float S = sqrt(pow(cur_x,2) + pow(cur_y,2));
    float SV = sqrt(pow(cur_x_vel-des_x_vel,2) + pow(cur_y_vel-des_y_vel,2));

    st_x = cur_x/S; st_y = cur_y/S;
    st_x_v = (cur_x_vel-des_x_vel)/SV; st_y_v = (cur_y_vel-des_y_vel)/SV;

	if (cur_z < 3)
    {
    	Real_z = cur_z;

	    for (int i = 0;i < 5;i++)
	    {
	    	cout << "3" << endl;
	    	ros::spinOnce();

	    	float S = sqrt(pow(cur_x,2) + pow(cur_y,2));

	    	st_x = cur_x/S; st_y = cur_y/S;
	    	
	    	Real_x = -0.1*(st_y) + 15.3;
	    	Real_y = -0.1*(st_x) + 19;
	    	
	    	desired_position << Real_x, Real_y, Real_z;

	    	mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);

	    	trajectory_pub.publish(trajectory_msg);

	    	sleep(2);
	    }

    	cout << "1" << endl;

        while (cur_z > 0.2)
        {
			Real_x = -(cur_y) + 15.3;
        	Real_y = -(cur_x)  + 19;
	    	Real_z = cur_z;

	    	desired_position << Real_x, Real_y, Real_z;

	    	mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);

	    	trajectory_pub.publish(trajectory_msg);

	    	cur_z = cur_z - 0.2;
	    	sleep(1);
        }

        for (int i = 0; i < 6; i++)
        {
        	actuator_msg.angular_velocities.push_back(0);
        }

        system("rosnode kill /firefly/lee_position_controller_node");

        motor_publisher.publish(actuator_msg);

        system("rosnode kill /firefly/waypoint_publisher");

    }
    else
    {
    	cout << "2" << endl;

    	// Translation
    	Real_x = -0.35*(st_y*0.5) - 0.8*st_y_v*0.5 + 15.3;
        Real_y = -0.4*(st_x*0.5) - 0.08*st_x_v*0.5 + 19;
    	Real_z = cur_z;

    	cout << "Real_x: " << Real_x << ",Real_y: " << Real_y << ",Real_z: " << Real_z << endl;

        desired_position << Real_x, Real_y, Real_z;

        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);

        trajectory_pub.publish(trajectory_msg);

        // Down
    	Real_z = (cur_z - 1.3)*0.5;

    	cout << "Real_x_1: " << Real_x << ",Real_y_1: " << Real_y << ",Real_z_1: " << Real_z << endl;

        desired_position << Real_x, Real_y, Real_z;

        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);

        trajectory_pub.publish(trajectory_msg);
    }

    sleep(0.001);
}
*/