# XIVO ROS Node

This is a ROS wrapper around [XIVO](https://github.com/ucla-vision/xivo) set up for integration into catkin environments with other packages. It uses XIVO as an external library.

**Inputs:**
- Monocular RGB images (`sensor_msgs/Image`) published to the `/cam0` topic.
- IMU data (`sensor_msgs/Imu`) published to the `/imu0` topic.
 
**Outputs:** (all are optional)
- Pose and covariance (`geometry_msgs/PoseWithCovarianceStamped`) published to `/xivo/pose`
- Map data (custom message type `FeatureMap`) published to `xivo/map`
- Pose and calibration state (custom message type `FullState`) published to `/xivo/fullstate`
- (For 2D navigation problems) Motion state (custom message type `MotionState2dNav`) published to `/xivo/twod_nav`



## Setup 

1. Download and build XIVO.
2. Set the environment variable `XIVO_ROOT` to the location of XIVO's repository.
3. Place this repository into the `src` folder of a catkin workspace.
4. Compile the entire catkin workspace using `catkin_make`.


## Examples

### Small Project Tango Example

Data was collected on a Lenovo Phab 2.

- Launch file `launch/xivo.launch`, `launch/xivo.xml`
- [Rosbag download link](https://www.dropbox.com/sh/0w5b7fglxf3li2l/AABAGYTU8QCq-vPuD-cqO4xta?dl=0)



### TUM VIO

Uses the input data of camera 0 and the IMU of the TUM VIO stereo rig. Tested on sequences room1-room6.

- Launch file `launch/tumvi.launch`, `launch/tumvi.xml`
- [Rosbag download link](https://vision.in.tum.de/data/datasets/visual-inertial-dataset)
