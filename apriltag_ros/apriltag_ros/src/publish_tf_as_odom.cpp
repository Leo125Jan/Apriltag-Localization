#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <iostream>
#include <string>
#include <cmath>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <tf/transform_broadcaster.h>


void bundleSelection(std::vector<geometry_msgs::TransformStamped> transforms_found, std::vector<geometry_msgs::TransformStamped>& transforms_selected);
void bundleFusion(std::vector<geometry_msgs::TransformStamped> transforms_selected, geometry_msgs::Pose& uav_pose_world);
void odomPublish(geometry_msgs::Pose uav_pose_world);
void stallUAV(double time);
void getMap2BundleTf(std::string source_frame, geometry_msgs::TransformStamped &source2target_transform);

bool jumpCheck(geometry_msgs::Pose uav_pose_world);
geometry_msgs::Pose poseTransform(geometry_msgs::Pose pose, std::string source_frame, std::string target_frame);
geometry_msgs::Pose averagePose(std::vector<geometry_msgs::Pose> all_poses_wrt_map);
std::vector<geometry_msgs::Pose> stdFilter(std::vector<geometry_msgs::Pose> all_poses_wrt_map);

geometry_msgs::Pose last_uav_pose_world;
int max_bundle_num = -1; // maximum number of bundle to consider to enter standard deviation filter
float opti_i, opti_j, opti_k, opti_w;
bool use_optiTrack = false; // use optiTrack's orientation
bool restart = false;
bool flag = false;
std::vector<std::string> bundle_names;

double epsilon = 0.3; // distance jump threshold
double stall_time = 0.1;
double initial_x = 0;
double initial_y = 0;
double initial_z = 0.15;
double desired_x = 0;
double desired_y = 0;
double desired_z = 1;

ros::Publisher odom_pub;
ros::Subscriber opti_sub, odom_sub, april_sub;

void optiTrackCallback(const geometry_msgs::PoseStamped::ConstPtr &msg){
  opti_i = (*msg).pose.orientation.x;
  opti_j = (*msg).pose.orientation.y;
  opti_k = (*msg).pose.orientation.z;
  opti_w = (*msg).pose.orientation.w;
}

void apriltagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg){
  if(msg->detections.size() > 0){
    flag = true;
    //ROS_WARN("true");
  }else{
    flag = false;
    //ROS_WARN("false");
  }

}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg){
  last_uav_pose_world.position.x = (*msg).pose.pose.orientation.x;
  last_uav_pose_world.position.y = (*msg).pose.pose.orientation.y;
  last_uav_pose_world.position.z = (*msg).pose.pose.orientation.z;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "tf_to_odom");
  ros::NodeHandle nh;
  ros::Rate rate(30);
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);	

  ros::V_string args;
  ros::removeROSArgs(argc, argv, args);

  odom_pub = nh.advertise<nav_msgs::Odometry>("/camera/pose_d", 30);
  opti_sub = nh.subscribe("/vrpn_client_node/MAV1/pose", 30, optiTrackCallback);
  odom_sub = nh.subscribe("/odometry_filtered", 30, odomCallback);
  april_sub = nh.subscribe("/downward/tag_detections", 30, apriltagCallback);
  bundle_names = args;
  
  last_uav_pose_world.position.x = initial_x;
  last_uav_pose_world.position.y = initial_y;
  last_uav_pose_world.position.z = initial_z;
  last_uav_pose_world.orientation.x = 0;
  last_uav_pose_world.orientation.y = 0;
  last_uav_pose_world.orientation.z = 0;
  last_uav_pose_world.orientation.w = 1.0;

  while (nh.ok()) {
    std::vector<geometry_msgs::TransformStamped> transforms_found, transforms_selected;
    geometry_msgs::Pose uav_pose_world;

    transforms_selected.clear();
    transforms_found.clear();
    // Read all Tf broadcasted by apriltag_ros
    for(int i = 1; i < bundle_names.size(); i++){
      try {
        // camera pose under bundle frame
        transforms_found.push_back(tfBuffer.lookupTransform(bundle_names.at(i), "camera_color_optical_frame", ros::Time(0))); // maybe camera_link is optical frame? //camera pose in bundle frame
        //ROS_WARN("Find Transform %d", i);
      } catch (tf2::TransformException &ex) {
        //ROS_WARN("%s", ex.what());
        //ROS_WARN("%d", i);
      }
    }

    // Select the <max_bundle_number> closest bundle to the camera 
    bundleSelection(transforms_found, transforms_selected);

    ROS_WARN("BBB");

    // Transform the camera pose in tag frame to map frame and averaging all measurement
    bundleFusion(transforms_selected, uav_pose_world);
    // check if an uav pose in world frame exists or not
    if(flag){
      // check if the computed pose is valid or not (steer jumping)
      if(jumpCheck(uav_pose_world)){
        ROS_WARN("PUBLISH RESULT");
        odomPublish(uav_pose_world);
        last_uav_pose_world = uav_pose_world;
        restart = false;
      }else{
        // stall the UAV to capture better detection
        ROS_ERROR("BAD RESULT");
        stallUAV(stall_time);
        restart = true;
      }
    }else{
      // stall the UAV to capture better detection
      ROS_ERROR("NO RESULT");
      ros::spinOnce();
      rate.sleep();
      continue;
    }
    flag = false;
    ros::spinOnce();
    rate.sleep();
  }
}

// Selecting <max_bundle_number> bundles with the smallest distance
void bundleSelection(std::vector<geometry_msgs::TransformStamped> transforms_found, std::vector<geometry_msgs::TransformStamped>& transforms_selected){
  std::vector<float> dist; // distance of camera and bundle
  std::vector<int> sorted_bundle; // sort the bundles according to their distance
  geometry_msgs::TransformStamped transformStamped;
  
  // iterate through all detected bundles and sort them in the order of their distance with camera
  for(int i = 0; i < transforms_found.size(); i++){
    float tmp;

    transformStamped = transforms_found[i];
    tmp = pow(transformStamped.transform.translation.x,2) + pow(transformStamped.transform.translation.y,2) + pow(transformStamped.transform.translation.z,2);

    if(i == 0){
      dist.push_back(tmp);
      sorted_bundle.push_back(i);
    }else{
      for(int j = 0; j < dist.size(); j++){
        if(tmp < dist[j]){
          dist.insert(dist.begin()+j,tmp);
          sorted_bundle.insert(sorted_bundle.begin()+j,i);
          break;
        }else if(j == (dist.size()-1)){
          dist.push_back(tmp);
          sorted_bundle.push_back(i);
          break;
        }
      }
    }
  }

  //std::cout << "sort bundle size" << sorted_bundle.size() << std::endl;
  if(max_bundle_num == -1) max_bundle_num = sorted_bundle.size();

  if(sorted_bundle.size()>0){
    for(int k, cnt; (k < sorted_bundle.size()) || (cnt < max_bundle_num); k++){
      // check if the z is jumping to negative, if so, then ignore it and find the next smallest
      if(transforms_found[sorted_bundle[k]].transform.translation.z >= 0){
        transforms_selected.push_back(transforms_found[sorted_bundle[k]]);
        cnt++;
      }
    }
  }

}

// Averaging the Pose from previously selected bundles
void bundleFusion(std::vector<geometry_msgs::TransformStamped> transforms_selected, geometry_msgs::Pose &uav_pose_world){
  // 1. find the inverse of transforms_selected to obtain camera pose w.r.t bundle frame (seems duplicated so didn't implemented)
  // 2. use the map to bundle static tf to obtain camera pose w.r.t map frame
  // 3. averaging all obtained position and orientation(quaternion can not be averaged)
  std::vector<geometry_msgs::Pose> all_poses_wrt_map;

  //std::cout << "selected size" << transforms_selected.size() << std::endl;
  if(transforms_selected.size()>0){
    for(int i=0; i < transforms_selected.size(); i++){
      geometry_msgs::Pose bundle_pose_wrt_camera, robot_pose_wrt_camera, robot_pose_wrt_bundle, robot_pose_wrt_map;

      bundle_pose_wrt_camera.position.x = transforms_selected[i].transform.translation.x;
      bundle_pose_wrt_camera.position.y = transforms_selected[i].transform.translation.y;
      bundle_pose_wrt_camera.position.z = transforms_selected[i].transform.translation.z;
      bundle_pose_wrt_camera.orientation.x = transforms_selected[i].transform.rotation.x;
      bundle_pose_wrt_camera.orientation.y = transforms_selected[i].transform.rotation.y;
      bundle_pose_wrt_camera.orientation.z = transforms_selected[i].transform.rotation.z;
      bundle_pose_wrt_camera.orientation.w = transforms_selected[i].transform.rotation.w;
      //std::cout << i << std::endl;
      //std::cout << "bundle_pose_wrt_camera" << bundle_pose_wrt_camera << std::endl;

      // transform from camera_link to base_link
      //robot_pose_wrt_camera = poseTransform(bundle_pose_wrt_camera, "camera_color_optical_frame", "base_link");
      //std::cout << "bundle_pose_wrt_robot" << robot_pose_wrt_camera << std::endl;

      // transform from base link frame to bundle frame
      //robot_pose_wrt_bundle = poseTransform(robot_pose_wrt_camera, "camera_color_optical_frame", transforms_selected[i].child_frame_id);
      //std::cout << "robot_pose_wrt_bundle" << robot_pose_wrt_bundle << std::endl;

      // transform the pose from bundle frame to map frame
      robot_pose_wrt_map = poseTransform(bundle_pose_wrt_camera, transforms_selected[i].header.frame_id, "map"); // not sure if the child frame id is the bundle's id???
      //std::cout << "robot_pose_wrt_map" << "\n" << robot_pose_wrt_map << std::endl;

      all_poses_wrt_map.push_back(robot_pose_wrt_map);
    }
    //std::cout << all_poses_wrt_map[0] << std::endl;
    //std::cout << "BB"<<all_poses_wrt_map.size() << std::endl;
  all_poses_wrt_map = stdFilter(all_poses_wrt_map);
  //std::cout << "AA"<<all_poses_wrt_map.size() << std::endl;

  uav_pose_world = averagePose(all_poses_wrt_map);

  }
}

geometry_msgs::Pose poseTransform(geometry_msgs::Pose pose_wrt_source_frame, std::string source_frame, std::string target_frame){
  geometry_msgs::Pose pose_wrt_target_frame;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped source2target_transform;

  getMap2BundleTf(source_frame, source2target_transform);
  tf2::doTransform(pose_wrt_source_frame, pose_wrt_target_frame, source2target_transform);
  return pose_wrt_target_frame;
}

geometry_msgs::Pose averagePose(std::vector<geometry_msgs::Pose> all_poses_wrt_map){
  float sum_x{0}, sum_y{0}, sum_z{0};
  geometry_msgs::Pose avg_pose;

  if(all_poses_wrt_map.size() > 0){
    for(int i = 0; i < all_poses_wrt_map.size(); i++){
      sum_x += all_poses_wrt_map[i].position.x;
      sum_y += all_poses_wrt_map[i].position.y;
      sum_z += all_poses_wrt_map[i].position.z;
    }

    avg_pose.position.x = sum_x / all_poses_wrt_map.size();
    avg_pose.position.y = sum_y / all_poses_wrt_map.size();
    avg_pose.position.z = sum_z / all_poses_wrt_map.size();

    if(use_optiTrack){
      avg_pose.orientation.x = opti_i;
      avg_pose.orientation.y = opti_j;
      avg_pose.orientation.z = opti_k;
      avg_pose.orientation.w = opti_w;
    }else{
      // maybe slerp averaging can be applied, currently picking the closest detection's orientation
      avg_pose.orientation.x = all_poses_wrt_map[0].orientation.x;
      avg_pose.orientation.y = all_poses_wrt_map[0].orientation.y;
      avg_pose.orientation.z = all_poses_wrt_map[0].orientation.z;
      avg_pose.orientation.w = all_poses_wrt_map[0].orientation.w;
    }
    //std::cout << "avg size" << sum_x << all_poses_wrt_map.size() << "avg_pose\n" << avg_pose << std::endl;
  }else{
    avg_pose.position.x = 999;
    avg_pose.position.y = 999;
    avg_pose.position.z = 999;
  }
  return avg_pose;
}

void odomPublish(geometry_msgs::Pose uav_pose_world){
    nav_msgs::Odometry odom;

    odom.header.stamp = ros::Time::now();
    odom.pose.pose.position.x = uav_pose_world.position.x;
    odom.pose.pose.position.y = uav_pose_world.position.y;
    odom.pose.pose.position.z = uav_pose_world.position.z;
    odom.pose.pose.orientation.w = uav_pose_world.orientation.w;
    odom.pose.pose.orientation.x = uav_pose_world.orientation.x;
    odom.pose.pose.orientation.y = uav_pose_world.orientation.y;
    odom.pose.pose.orientation.z = uav_pose_world.orientation.z;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "camera_color_optical_frame";

    // //send the transform
    //odom_broadcaster.sendTransform(odom_trans);

    odom_pub.publish(odom);
    std::cout << uav_pose_world << std::endl;
}

void stallUAV(double time){
  geometry_msgs::Pose uav_pose_world;

  ROS_WARN("====No Valid Transform is Found====");
  ROS_WARN("====Stopping to capture better prediction====");
  // need to change to position command given to uav
  
  uav_pose_world.position.x = 0;
  uav_pose_world.position.y = 0;
  uav_pose_world.position.z = desired_z;
  uav_pose_world.orientation.x = 0;
  uav_pose_world.orientation.y = 0;
  uav_pose_world.orientation.z = 0;
  uav_pose_world.orientation.w = 1.0;

  // stall the uav by illucinated it being at destination
  ros::Time start_time = ros::Time::now();
  ros::Duration timeout(time); // Timeout of 0.5 seconds
  while(ros::Time::now() - start_time < timeout) {
    odomPublish(uav_pose_world);
    ros::spinOnce();
  }
}

bool jumpCheck(geometry_msgs::Pose uav_pose_world){
  float delta_x, delta_y;

  delta_x = abs(last_uav_pose_world.position.x - uav_pose_world.position.x);
  delta_y = abs(last_uav_pose_world.position.y - uav_pose_world.position.y);
  
  if((delta_x > epsilon || delta_y > epsilon) && !restart){
    return false;
  }else{
    return true;
  }
  
}

void getMap2BundleTf(std::string source_frame, geometry_msgs::TransformStamped &source2target_transform){
  std::vector<float> pose;
  ros::param::get("map2tag/"+source_frame, pose);
  source2target_transform.transform.translation.x = pose[0];
  source2target_transform.transform.translation.y = pose[1];
  source2target_transform.transform.translation.z = pose[2];
  source2target_transform.transform.rotation.x = pose[3];
  source2target_transform.transform.rotation.y = pose[4];
  source2target_transform.transform.rotation.z = pose[5];
  source2target_transform.transform.rotation.w = pose[6];

  //std::cout << pose[0] << std::endl;
}

// Eliminate the outlier whose is one standard deviation away from the mean of all considered measurements
std::vector<geometry_msgs::Pose> stdFilter(std::vector<geometry_msgs::Pose> all_poses_wrt_map){
  float sum_x{0}, sum_y{0}, sum_z{0}, avg_x{0}, avg_y{0}, avg_z{0}, std_x{0}, std_y{0}, std_z{0};
  std::vector<geometry_msgs::Pose> all_poses_wrt_map_filtered;

  for(int i = 0; i < all_poses_wrt_map.size(); i++){
    sum_x += all_poses_wrt_map[i].position.x;
    sum_y += all_poses_wrt_map[i].position.y;
    sum_z += all_poses_wrt_map[i].position.z;
  }

  avg_x = sum_x / all_poses_wrt_map.size();
  avg_y = sum_y / all_poses_wrt_map.size();
  avg_z = sum_z / all_poses_wrt_map.size();

  sum_x = 0;
  sum_y = 0;
  sum_z = 0;
  for(int i = 0; i < all_poses_wrt_map.size(); i++){
    sum_x += pow(all_poses_wrt_map[i].position.x - avg_x, 2);
    sum_y += pow(all_poses_wrt_map[i].position.y - avg_y, 2);
    sum_z += pow(all_poses_wrt_map[i].position.z - avg_z, 2);
  }

  std_x = sqrt(sum_x/all_poses_wrt_map.size());
  std_y = sqrt(sum_y/all_poses_wrt_map.size());
  std_z = sqrt(sum_z/all_poses_wrt_map.size());

  //std::cout << "aaaa"<< sum_x << std::endl;
  for(int i = 0; i < all_poses_wrt_map.size(); i++){
    bool cond_x = true;
    bool cond_y = true;
    bool cond_z = true;

    cond_x = abs(all_poses_wrt_map[i].position.x - avg_x) > 1.5*std_x;
    cond_y = abs(all_poses_wrt_map[i].position.y - avg_y) > 1.5*std_y;
    cond_z = abs(all_poses_wrt_map[i].position.z - avg_z) > 1.5*std_z;

    if(!cond_x && !cond_y && !cond_z){
      //ROS_WARN("PUSH TO STD");
      all_poses_wrt_map_filtered.push_back(all_poses_wrt_map[i]);
    }
  }

  return all_poses_wrt_map_filtered;
}

