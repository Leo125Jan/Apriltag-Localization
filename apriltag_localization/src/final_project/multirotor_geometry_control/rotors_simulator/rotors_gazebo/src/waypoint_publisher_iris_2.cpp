#include <cmath>
#include <fstream>
#include <unistd.h>
#include <iostream>
#include <ros/ros.h>
#include <Eigen/Core>
#include <nav_msgs/Path.h>
#include <std_msgs/Int64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

using namespace std;

geometry_msgs::Pose f_ode;

float f_ode_x, f_ode_y, f_ode_z;
float C = 1;

void f_ode_Callback(const geometry_msgs::Pose::ConstPtr &msg_f)
{
	C = C + 1;

	if (C == 3)
	{
		f_ode = *msg_f;

		f_ode_x = f_ode.position.x;
		f_ode_y = f_ode.position.y;
	    f_ode_z = f_ode.position.z;
	}
}

geometry_msgs::Pose t_ode;

float t_ode_x, t_ode_y, t_ode_z;
float b = 1;

void t_ode_Callback(const geometry_msgs::Pose::ConstPtr &msg_t)
{
	b = b + 1;

	if (b == 3)
	{
		t_ode = *msg_t;

		t_ode_x = t_ode.position.x;
		t_ode_y = t_ode.position.y;
	    t_ode_z = t_ode.position.z;
	}
}

apriltag_ros::AprilTagDetectionArray Firefly_pose;

float x, y, z, Real_x, Real_y, Real_z;
float cur_x, cur_y, cur_z;

void LandCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
	Firefly_pose = *msg;

	cur_x = Firefly_pose.detections[0].pose.pose.pose.position.x;
	cur_y = Firefly_pose.detections[0].pose.pose.pose.position.y;
    cur_z = Firefly_pose.detections[0].pose.pose.pose.position.z;

    cout << "cur_x: " << cur_x << ",cur_y: " << cur_y << ",cur_z: " << cur_z << endl;
}

geometry_msgs::Pose i_ode;

float i_ode_x, i_ode_y, i_ode_z;

void i_ode_Callback(const geometry_msgs::Pose::ConstPtr &msg_i)
{
	i_ode = *msg_i;

	i_ode_x = i_ode.position.x;
	i_ode_y = i_ode.position.y;
    i_ode_z = i_ode.position.z;
}

float Standard(void)
{
	float st_x, st_y, S;

	S = sqrt(pow(cur_x,2) + pow(cur_y,2));
	st_x = cur_x/S; st_y = cur_y/S;

	return st_x, st_y;
}

void Move_to_Neo(ros::NodeHandle &nh, ros::Publisher &trajectory_pub, const float DEG_2_RAD, double desired_yaw
					, float &f_x, float &f_y)
{
	// Move to firefly
	ROS_INFO("Waitting for Neo11 position message");

	// Node setting
	ros::Subscriber f_ode_sub;

	// Variable setting
	Eigen::Vector3d desired_position;
	trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
	f_ode_sub = nh.subscribe("/neo11/odometry_sensor1/pose", 1000, f_ode_Callback);

	float Real_x_h, Real_y_h, Real_z_h;

	// WaitforMessage setting
	boost::shared_ptr<geometry_msgs::Pose const> sharedEdge;
	sharedEdge = ros::topic::waitForMessage<geometry_msgs::Pose>("/neo11/odometry_sensor1/pose", nh);

	sleep(1);

	if(sharedEdge != NULL)
	{
		ROS_INFO("Getting Neo11 position message");
	}

	sleep(2);

	for (int i = 0; i < 100; i++)
	{
		ros::spinOnce();

		if (C == 3)
		{
			break;
		}
	}
	
	cout << "ode_x: " << f_ode_x << ", ode_y: " << f_ode_y << ", ode_z: " << f_ode_z << endl;

	Real_x_h = f_ode_x - 0.1;
	Real_y_h = f_ode_y - 0.1;
	Real_z_h = f_ode_z + 5;

	desired_position << Real_x_h, Real_y_h, Real_z_h;

	mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);

	for (int i = 0; i < 5; i++)
	{
		trajectory_pub.publish(trajectory_msg);
	}

	f_x = Real_x_h; f_y = Real_y_h;
}

void Landing_on_Neo(ros::NodeHandle &nh, ros::Publisher &trajectory_pub, const float DEG_2_RAD, double desired_yaw
			, float f_x, float f_y)
{
	// //Landing
	ROS_INFO("Prepare for Landing");

	sleep(1);

	// Node setting
	ros::Publisher motor_publisher;
	ros::Publisher neo11_pub;
	ros::Subscriber i_ode_sub;
	ros::Subscriber land_sub;

	// Variable setting
	mav_msgs::Actuators actuator_msg;
	Eigen::Vector3d desired_position;
	trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;

	land_sub = nh.subscribe("/tag_detections", 1000, LandCallback);
	neo11_pub = nh.advertise<std_msgs::Int64>("/neo11/waypoint_2", 1000);
	i_ode_sub = nh.subscribe("/iris2/odometry_sensor1/pose", 1000, i_ode_Callback);
	motor_publisher = nh.advertise<mav_msgs::Actuators>("/iris2/command/motor_speed", 10);

	// WaitforMessage setting
	boost::shared_ptr<std_msgs::Int64 const> Edge;	
	boost::shared_ptr<apriltag_ros::AprilTagDetectionArray const> sharedEdge;
    sharedEdge = ros::topic::waitForMessage<apriltag_ros::AprilTagDetectionArray>("/tag_detections");
	
    if(sharedEdge != NULL)
	{
		ros::spinOnce();
	}

    while (ros::ok())
	{

		if (cur_z*0.5 < 6.5)
	    {
	    	float k_x, k_y;

		    for (int i = 0;i < 10;i++)
		    {
		    	cout << "3" << endl;

		    	ros::spinOnce();

		    	cur_x, cur_y = Standard();

		    	if (abs(cur_x) > 0.15)
		    	{
		    		k_x = -0.2;
		    	}
		    	else
		    	{
		    		k_x = -0.15;
		    	}

		    	if (abs(cur_y) > 0.3)
		    	{
		    		k_y = -0.2;
		    	}
		    	else
		    	{
		    		k_y = -0.15;
		    	}

		    	Real_x = k_x*(cur_y) - 0.08 + i_ode_x;
		    	Real_y = k_y*(cur_x) + 0.03 + i_ode_y;
		    	
		    	desired_position << Real_x, Real_y, Real_z;

		    	mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);

		    	trajectory_pub.publish(trajectory_msg);

			    sleep(1);
		    }

	    	cout << "1" << endl;
	    	cur_z = f_ode_z + cur_z*0.2;

	        while (cur_z > f_ode_z + 0.1)
	        {
				// Real_x = -0.1*(cur_y) + i_ode_x;
	        	// Real_y = -0.1*(cur_x) + i_ode_y;
		    	Real_z = cur_z;

		    	desired_position << Real_x, Real_y, Real_z;

		    	mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);

		    	trajectory_pub.publish(trajectory_msg);

		    	cur_z = cur_z - 0.08;
		    	sleep(1);
	        }

	        ROS_INFO("Complete Landing");

	        for (int i = 0; i < 6; i++)
	        {
	        	actuator_msg.angular_velocities.push_back(0);
	        }

	        ROS_INFO("Motors stop");

        	desired_position << Real_x, Real_y, -10;

	    	mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);

	    	trajectory_pub.publish(trajectory_msg);

	        // system("rosnode kill /iris/lee_position_controller_node");

	        // motor_publisher.publish(actuator_msg);

	        system("rosrun gazebo_ros_link_attacher attach_t.py");

	        break;

	    }
	    else
	    {
	    	cout << "2" << endl;

	    	// Translation
	    	cur_x, cur_y = Standard(); 

	    	Real_x = -0.3*(cur_y) + f_x;
	        Real_y = -0.19*(cur_x) + f_y;
	    	Real_z = cur_z;

	    	cout << "Real_x: " << Real_x << ",Real_y: " << Real_y << ",Real_z: " << Real_z << endl;

	        desired_position << Real_x, Real_y, Real_z;

	        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);

	        trajectory_pub.publish(trajectory_msg);

	        // Down
	    	Real_z = cur_z*0.5;

	    	cout << "Real_x_1: " << Real_x << ",Real_y_1: " << Real_y << ",Real_z_1: " << Real_z << endl;

	        desired_position << Real_x, Real_y, Real_z;

	        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);

	        trajectory_pub.publish(trajectory_msg);
	    }

	    ros::spinOnce();
	    sleep(0.5);
    }

    sleep(3);

    ROS_INFO("Publish neo11 second waypoint");

	std_msgs::Int64 Go;
    Go.data = 1;

    neo11_pub.publish(Go);

    sleep(5);

    ROS_INFO("wait for detach");

    Edge = ros::topic::waitForMessage<std_msgs::Int64>("/iris2/detach");

	if(Edge != NULL)
	{
		ROS_INFO("Get detach message");
	}

    sleep(3);

    system("rosrun gazebo_ros_link_attacher detach_t.py");
}

void Taking_off(ros::NodeHandle &nh, ros::Publisher &trajectory_pub, const float DEG_2_RAD, double desired_yaw
				, float &f_x, float &f_y)
{
	// Move to charge
	ROS_INFO("Take off");

    sleep(1);

    // Node setting
    ros::Subscriber t_ode_sub;

    // Variable setting
    Eigen::Vector3d desired_position;
	trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    t_ode_sub = nh.subscribe("/iris2/odometry_sensor1/pose", 1000, t_ode_Callback);

    float Real_x_h, Real_y_h, Real_z_h;
    
    while (ros::ok())
    {
    	ros::spinOnce();

    	if (b == 3)
    	{
    		break;
    	}	
    }

    sleep(2);

   	Real_x_h = t_ode_x;
	Real_y_h = t_ode_y;
	Real_z_h = t_ode_z + 5;

	cout << "t_ode_x: " << t_ode_x << ", t_ode_y: " << t_ode_y << ", t_ode_z: " << t_ode_z << endl;

	desired_position << Real_x_h, Real_y_h, Real_z_h;

	mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);

	trajectory_pub.publish(trajectory_msg);

	sleep(3);

	float decend_x =  (13 - Real_x_h)/10;
	float decend_y =  (13 - Real_y_h)/10;
	float decend_z =  Real_z_h/10;

	for (int i = 1; i <= 10; i++)
	{
		Real_x_h = Real_x_h + decend_x;
		Real_y_h = Real_y_h + decend_y;
		Real_z_h = Real_z_h - (decend_z/2);

		desired_position << Real_x_h, Real_y_h, Real_z_h;

		mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);

		trajectory_pub.publish(trajectory_msg);

		sleep(1);
	}

	f_x = Real_x_h; f_y = Real_y_h;
}

void Landing_on_charge_site(ros::NodeHandle &nh, ros::Publisher &trajectory_pub, const float DEG_2_RAD,
							double desired_yaw, float f_x, float f_y)
{
	//Landing
	ROS_INFO("Prepare for Landing");

	// Node setting
	ros::Publisher motor_publisher;
	ros::Subscriber i_ode_sub;
	ros::Subscriber land_sub;

	// Variable setting
	mav_msgs::Actuators actuator_msg;
	Eigen::Vector3d desired_position;
	trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;

	land_sub = nh.subscribe("/tag_detections", 1000, LandCallback);
	i_ode_sub = nh.subscribe("/iris2/odometry_sensor1/pose", 1000, i_ode_Callback);
	motor_publisher = nh.advertise<mav_msgs::Actuators>("/iris2/command/motor_speed", 10);

	sleep(2);

	// WaitforMessage setting
	boost::shared_ptr<apriltag_ros::AprilTagDetectionArray const> sharedEdge;
	sharedEdge = ros::topic::waitForMessage<apriltag_ros::AprilTagDetectionArray>("/tag_detections");

    if(sharedEdge != NULL)
	{
		ros::spinOnce();
	}

    while (ros::ok())
	{
		ros::spinOnce();

		if (cur_z*0.7 < 3)
	    {
	    	Real_z = cur_z - 1.5;

	    	float k_x, k_y;

		    for (int i = 0;i < 10;i++)
		    {
		    	cout << "3" << endl;

		    	ros::spinOnce();

		    	cur_x, cur_y = Standard();

		    	if (abs(cur_x) > 0.15)
		    	{
		    		k_x = -0.2;
		    	}
		    	else
		    	{
		    		k_x = -0.14;
		    	}

		    	if (abs(cur_y) > 0.5)
		    	{
		    		k_y = -0.3;
		    	}
		    	else
		    	{
		    		k_y = -0.2;
		    	}

		    	Real_x = k_x*(cur_y) - 0.08 + i_ode_x;
		    	Real_y = k_y*(cur_x) + 0.03 + i_ode_y;
		    	
		    	desired_position << Real_x, Real_y, Real_z;

		    	mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);

		    	trajectory_pub.publish(trajectory_msg);

			    sleep(1);
		    }

	    	cout << "1" << endl;

	    	cur_z = cur_z;

	        while (cur_z*0.7 > 0.5)
	        {
		    	Real_z = cur_z - 0.8;

		    	desired_position << Real_x, Real_y, Real_z;

		    	mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);

		    	trajectory_pub.publish(trajectory_msg);

		    	cur_z = cur_z*0.7 - 0.08;
		    	sleep(1);
	        }

	        ROS_INFO("Complete Landing");

	        for (int i = 0; i < 6; i++)
	        {
	        	actuator_msg.angular_velocities.push_back(0);
	        }

	        ROS_INFO("Motors stop");

	        system("rosnode kill /iris2/lee_position_controller_node");

	        motor_publisher.publish(actuator_msg);

	        break;

	    }
	    else
	    {
	    	cout << "2" << endl;

	    	// Translation
	    	cur_x, cur_y = Standard(); 

	    	Real_x = -0.3*(cur_y) + f_x;
	        Real_y = -0.19*(cur_x) + f_y;
	    	Real_z = cur_z;

	    	cout << "Real_x: " << Real_x << ",Real_y: " << Real_y << ",Real_z: " << Real_z << endl;

	        desired_position << Real_x, Real_y, Real_z;

	        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);

	        trajectory_pub.publish(trajectory_msg);

	        // Down
	    	Real_z = cur_z*0.7 - 1.3;

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

	// Waiting for iris finish
	ROS_INFO("Waiting for iris finish");

	boost::shared_ptr<std_msgs::Int64 const> sharedEdge;	
	sharedEdge = ros::topic::waitForMessage<std_msgs::Int64>("/iris2/go");

	if(sharedEdge != NULL)
	{
		ROS_INFO("Get Start message");
	}

	sleep(3);

	float f_x, f_y;

	Move_to_Neo(nh, trajectory_pub, DEG_2_RAD, desired_yaw, f_x, f_y);

	sleep(1);

	Landing_on_Neo(nh, trajectory_pub, DEG_2_RAD, desired_yaw, f_x, f_y);

    sleep(3);

    Taking_off(nh, trajectory_pub, DEG_2_RAD, desired_yaw, f_x, f_y);

    sleep(1);

    Landing_on_charge_site(nh, trajectory_pub, DEG_2_RAD, desired_yaw, f_x, f_y);

    return 0;
}