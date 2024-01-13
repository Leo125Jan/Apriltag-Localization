# Apriltag-Localization

## About the Project

1. Utilizing AprilTag for autonomous positioning of drones, completing factory patrol tasks.
2. Preliminary task: First, conduct tests in a simulated environment (Gazebo), then proceed to physical experiments.

## Built With

### Simulation Environmrnt
* ROS & Gazebo

### Apriltag
* [Apriltags](https://optitag.io/blogs/news/using-your-apriltag-with-ros)
* [Apriltag_ros](https://github.com/AprilRobotics/apriltag_ros)

### D435i Camera
* [Realsense_Gazebo_Plugin](https://github.com/pal-robotics/realsense_gazebo_plugin)
* [IntelRealSense](https://github.com/IntelRealSense/realsense-ros)

### Robot Localization
We use following package, it employ ekf/ukf to fuse the data from imu and apriltags.
* [Robot_Localization](https://github.com/cra-ros-pkg/robot_localization/tree/melodic-devel)
