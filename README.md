# Apriltag-Localization

## About the Project

1. Utilizing AprilTag for autonomous positioning of drones, completing factory patrol tasks.
2. Preliminary task: First, conduct tests in a simulated environment (Gazebo), then proceed to physical experiments.

## Built With

### Simulation
* ROS & Gazebo
* [Rotor Simulator](https://github.com/ethz-asl/rotors_simulator)

### Experiment
* NCRL flight controller

### Apriltag
* [Apriltags](https://optitag.io/blogs/news/using-your-apriltag-with-ros)
* [Apriltag_ros](https://github.com/AprilRobotics/apriltag_ros)

### D435i Camera
* [Realsense_Gazebo_Plugin](https://github.com/pal-robotics/realsense_gazebo_plugin)
* [IntelRealSense](https://github.com/IntelRealSense/realsense-ros)

### Robot Localization
We use the following package, which employs EKF/UKF to fuse the data from the IMU and camera detecting AprilTags.
* [Robot_Localization](https://github.com/cra-ros-pkg/robot_localization/tree/melodic-devel)

## Usage

### Simulation

* UAV with forward and downward d435i camera
![fbAIWoS](https://github.com/Leo125Jan/Apriltag-Localization/assets/98295556/e85fcb13-1cd0-46cd-9d24-a04931d4afb0)

* Environment for evaluation
![3qKaYNB](https://github.com/Leo125Jan/Apriltag-Localization/assets/98295556/174385b8-ebc1-4714-bf0d-e29222dfa3a6)

* Comparison of ground truth and localization by AprilTags

Our goal is to test the accuracy of localization using AprilTags in simulation environment.

<a href="http://www.youtube.com/watch?v=-AoKKMnz1AA" target="_blank"><img src="http://img.youtube.com/vi/-AoKKMnz1AA/0.jpg" 
width="480" height="270" border="10" /></a>

### Implement

* Comparison of OptiTrack localization and AprilTags localization

Our goal is to test the accuracy of localization using AprilTags in reality.

<a href="http://www.youtube.com/watch?v=7sUJfCsmZnY" target="_blank"><img src="http://img.youtube.com/vi/7sUJfCsmZnY/0.jpg" 
width="480" height="270" border="10" /></a>

* Performing flight using AprilTag localization

Subsequently, we relied solely on AprilTags localization for UAV to take off. However, after takeoff, UAV immediately deviated in a certain direction and eventually left the range of AprilTags. We speculate that the reason might be that the camera may not detect the AprilTags during flight, causing the position estimation to rely only on the IMU. Over time, the IMU accumulates errors, leading to inaccurate position estimation and instability of the UAV.

This project has ultimately progressed to this step.

