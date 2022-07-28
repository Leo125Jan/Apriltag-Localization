#
#include <ros/ros.h>
#include <geometric_controller.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/LinkStates.h>
#include <tf/transform_datatypes.h>
#include <qptrajectory.h>
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf2/transform_datatypes.h>
#include <tf/transform_datatypes.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/callback_queue.h>
#include <std_msgs/Int8.h>
#include "apriltag_ros/AprilTagDetectionArray.h"

#define normal
#define PI 3.14159


gazebo_msgs::ModelStates model_states;
float vir_x, vir_y,vir_z,vx,vy,vz,ax,ay,az;
unsigned int tick=0;
unsigned int counter = 0;
bool flag = false;
mavros_msgs::State current_state;
gazebo_msgs::LinkStates link_states;
geometry_msgs::PoseStamped desired_pose;
nav_msgs::Path UAV_path;
geometry_msgs::PointStamped ugv_pos;
geometry_msgs::PointStamped uav_pos;
geometry_msgs::PoseWithCovariance apriltag_pose;
geometry_msgs::Point target_pos;
geometry_msgs::Point track_pos;
std::vector<float> path_points_x{0.0};
std::vector<float> path_points_y{0.0};
std::vector<float> path_points_z{0.0};
std::vector<float> path_points_yaw{0.0};

int path_length{0};
double tt;
double r = 2;
double T = 280*M_PI;
int mode = 0;
int clk = 0;
int path_valid = 0;
double error = 0.0;
trajectory_profile start, goal;


ros::Publisher start_pose_pub, traj_pub, goal_pub, mode_pub;
ros::Subscriber wpSub, UAV_pos_sub, mode_sub, target_sub, ugv_sub, track_sub;


void setStart(){

	geometry_msgs::PoseWithCovarianceStamped posec;

	posec.pose.pose.position.x = uav_pos.point.x;
	posec.pose.pose.position.y = uav_pos.point.y;
	posec.pose.pose.orientation.z =0.671442089592;
	posec.pose.pose.orientation.w =0.741057029064;
	posec.header.frame_id = "map";
	posec.header.stamp = ros::Time::now();
	start_pose_pub.publish(posec);

}

void uavPosCallback(const geometry_msgs::PointStamped::ConstPtr& pos){
	uav_pos = *pos;
}

void square_path(void)
{
	path_points_x.clear();
	path_points_y.clear();
	path_points_z.clear();

	path_valid = 1;
	int point = 100;
	path_length = point*4;

	for(int i = 1; i <= point; i++)
	{
		path_points_x.push_back(-6 + 0.07*i);
		path_points_y.push_back(-0.5);
		path_points_z.push_back(2);
	}

	for(int i = 1; i <= point; i++)
	{
		path_points_x.push_back(1);
		path_points_y.push_back(-0.5 - 0.055*i);
		path_points_z.push_back(2);
	}

	for(int i = 1; i <= point; i++)
	{
		path_points_x.push_back(1 - 0.07*i);
		path_points_y.push_back(-6);
		path_points_z.push_back(2);
	}

	for(int i = 1; i <= point; i++)
	{
		path_points_x.push_back(-6);
		path_points_y.push_back(-6 + 0.055*i);
		path_points_z.push_back(2);
	}
}		
void manualGoalMode()
{
	std::string ns;
	ns = ros::this_node::getNamespace();
	std::cout << ns << std::endl;
	ROS_WARN("====Entering Setting Manual Goal====");
	double dt = 25.0;
	ros::Rate loop_rate(dt);

	qptrajectory plan;
	path_def path;
	std::vector<trajectory_profile> data;
	trajectory_msgs::MultiDOFJointTrajectoryPoint traj;
	double sample = 0.05;

	geometry_msgs::Transform transform;
	geometry_msgs::Twist twist;

	transform.translation.x = 0;
	transform.translation.y = 0;
	transform.translation.z = 0;
	transform.rotation.x = 0;
	transform.rotation.y = 0;
	transform.rotation.z = 0;
	transform.rotation.w = 0;
	twist.linear.x = 0;
	twist.linear.y = 0;
	twist.linear.z = 0;
	twist.angular.x = 0;
	twist.angular.y = 0;
	twist.angular.z = 0;

	traj.transforms.push_back(transform);
	traj.velocities.push_back(twist);
	traj.accelerations.push_back(twist);

	while(ros::ok())
	{
		if(path_length > 0)
		{
			
			ROS_INFO("Start solving trajectory...");
			start.pos << path_points_x[0], path_points_y[0], 2;
			goal.pos << path_points_x[path_length-1], path_points_y[path_length-1], 2;
			double total_dist = (start.pos - goal.pos).norm();
			double avg_v = 0.5;
			double total_time = (total_dist / avg_v);

			for(unsigned int k=2; k < path_length; k++)
			{
				if(k < path_length-1 || k == path_length-1)
				{
					trajectory_profile p1, p2;
					p1.pos << path_points_x[k], path_points_y[k], 2;
			
					p2.pos << path_points_x[k+1], path_points_y[k+1], 2;

					// std::cout << p1.pos << std::endl;
					double dist = (p1.pos - p2.pos).norm();

					// Time interval should be further adjust, small value will lead to qp failure
					double time_interval = (dist/total_dist) * total_time;
					
					path.push_back(segments(p1, p2, time_interval));
				}
			}
		
			std::cout << "Path size:" << path.size() << std::endl;
			data = plan.get_profile(path, path.size(), sample);

			ROS_INFO("Trajectory Planning finished, clearing path...");

			path_length = 0;
			path.clear();
		}
		else
		{
			if(clk%50 == 0) ROS_INFO("No path received...");
		}
		
		if(data.size() > 0)
		{
			if((tick>data.size()))
			{

				vir_x = goal.pos(0);
				vir_y = goal.pos(1);
				vir_z = goal.pos(2);
				vx = 0;
				vy = 0;
				vz = 0;
				ax = 0;
				ay = 0;
				az = 0;
			}
			else
			{
				vir_x = data[tick].pos(0);
				vir_y = data[tick].pos(1);
				vir_z = 2;
				vx = data[tick].vel(0);
				vy = data[tick].vel(1);
				vz = data[tick].vel(2);
				ax = data[tick].acc(0);
				ay = data[tick].acc(1);
				az = data[tick].acc(2);
				tick++;

				double t = ros::Time::now().toSec();
			}

			traj.transforms[0].translation.x = vir_x;
			traj.transforms[0].translation.y = vir_y;
			traj.transforms[0].translation.z = vir_z;
			traj.velocities[0].linear.x = vx;
			traj.velocities[0].linear.y = vy;
			traj.velocities[0].linear.z = vz;
			traj.accelerations[0].linear.x = ax;
			traj.accelerations[0].linear.y = ay;
			traj.accelerations[0].linear.z = az;
			traj.transforms[0].rotation.w = 0;
			traj.transforms[0].rotation.x = 0;
			traj.transforms[0].rotation.y = 0;
			traj.transforms[0].rotation.z = 0;

			traj_pub.publish(traj);
			
			if(tick == 1) ROS_INFO("Trajectory published...");			
		}
		
		clk++;
		if(clk>100) clk = 0;
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	return;
}

int main(int argc, char **argv)
{	
	
	ros::init(argc, argv, "geo");
	ros::NodeHandle nh;

	traj_pub= nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("desired_trajectory", 10);
	goal_pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal",1000);
	mode_pub = nh.advertise<std_msgs::Int8>("mode",1);
	start_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose",1 );

	std::string ns;
	double dt = 10.0;
	ros::Rate loop_rate(dt);
	int clk1 = 0;

	ns = ros::this_node::getNamespace();


	square_path();
	manualGoalMode();
	ROS_WARN("====Quitting Manual Goal Mode====");

	ros::spinOnce();

}