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
We use the following package, which employs EKF/UKF to fuse the data from the IMU and camera detecting AprilTags.
* [Robot_Localization](https://github.com/cra-ros-pkg/robot_localization/tree/melodic-devel)

## Usage

### Simulation

* UAV with forward and downward d435i camera
<a><img src="https://user-images.githubusercontent.com/98295556/42abefe9-42c8-4034-826d-629ff589029f.gif" 
width="640" height="360" border="10" /></a>

* Environment for evaluation
<a><img src="https://user-images.githubusercontent.com/assets/98295556/174385b8-ebc1-4714-bf0d-e29222dfa3a6.png" 
width="640" height="360" border="10" /></a>
![3qKaYNB](https://github.com/Leo125Jan/Apriltag-Localization/assets/98295556/174385b8-ebc1-4714-bf0d-e29222dfa3a6)

* Comparison of ground truth and localization by AprilTags

Our goal is to test the accuracy of loc

alization using AprilTags in simulation environment.
<a href="http://www.youtube.com/watch?v=-AoKKMnz1AA" target="_blank"><img src="http://img.youtube.com/vi/-AoKKMnz1AA/0.jpg" 
width="480" height="270" border="10" /></a>

### Implement

* Comparison of OptiTracl localization and AprilTags localization

Our goal is to test the accuracy of localization using AprilTags in reality.

<a href="http://www.youtube.com/watch?v=7sUJfCsmZnY" target="_blank"><img src="http://img.youtube.com/vi/7sUJfCsmZnY/0.jpg" 
width="480" height="270" border="10" /></a>


