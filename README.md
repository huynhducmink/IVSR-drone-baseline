# IVSR drone baseline
# A full autonomous drone navigation system with photo-realistic simulation environment

## Introduction



## Instalation
The simulation set is configured and verified to work on Ubuntu 20.04

Every ros package can be installed into the same workspace so that you won't have to manually source every workspace for each command
Eg:
```
sample_folder/flightmare_ws/src/(ALL ROS PACKAGE HERE)
sample_folder/PX4-Autopilot/
```

**Install ROS Noetic**

http://wiki.ros.org/noetic/Installation/Ubuntu

**Install PX4 autopilot**

https://docs.px4.io/main/en/dev_setup/building_px4.html

**Install MAVROS and GeographicLib**

https://github.com/mavlink/mavros/blob/master/mavros/README.md#installation

**Change parameter setting in QGroundControl:**

- Download QGroundControl from http://qgroundcontrol.com/
- Start PX4
```
roslaunch px4 mavros_posix_sitl.launch
```
- Find parameter COM_RCL_EXCEPT in vehicle setting and set to 4 (Offboard mode)

**Install Flightmare**

- https://flightmare.readthedocs.io/en/latest/getting_started/quick_start.html#quick-start
- After finish installing Flightmare, remove flightros folder in flightmare folder and replace with the following:
```
git clone https://github.com/huynhducmink/flightros.git
```

**Install VINS-FUSION**


- Install Ceres-solver 2.1.0 from http://ceres-solver.org/installation.html

Ceres don't need to install in ROS workspace, it will be installed to root. Also only version 2.1.0 will work, 2.2.0 won't

- Install VINS-Fusion

```
git clone https://github.com/huynhducmink/VINS-Fusion
```

**Install Ewok**
```
sudo apt-get install git python python3-matplotlib python3-numpy libeigen3-dev libgoogle-glog-dev libatlas-base-dev libsuitesparse-dev protobuf-compiler libnlopt-dev libnlopt-cxx-dev ros-noetic-octomap ros-noetic-octomap-ros ros-noetic-octomap-msgs ros-noetic-tf-conversions ros-noetic-eigen-conversions

sudo apt install ros-noetic-sophus

git clone https://github.com/huynhducmink/ewok_optimization_DN.git
```

**Install control package**
```
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
git clone https://github.com/ethz-asl/eigen_checks.git
git clone https://github.com/ethz-asl/nlopt.git
git clone https://github.com/ethz-asl/glog_catkin.git
git clone https://github.com/ethz-asl/yaml_cpp_catkin.git
git clone https://github.com/ethz-asl/mav_trajectory_generation.git
git clone https://github.com/huynhducmink/offboard
```

**Install VIO utilities package**

This package provides transformation, change topics to different format, visualize flight path in RVIZ,...
```
git clone https://github.com/huynhducmink/Visual_odometry_utilities
```

**Install MSF**
```
git clone https://github.com/huynhducmink/ethzasl_msf
```

**Build the workspace**
```
catkin build
```

**Edit .bashrc file**
After finishing install all of the above packages, edit the .bashrc file and add the lines like bellow if you haven't already:

The order is important, if you put the PX4 part before the Flightmare part it won't work

```
source /opt/ros/noetic/setup.bash
export FLIGHTMARE_WS_PATH=(ADD PATH TO THE ROS WORKSPACE)
source $FLIGHTMARE_WS_PATH/devel/setup.bash
export FLIGHTMARE_PATH=$FLIGHTMARE_WS_PATH/src/flightmare

export PX4_INSTALL_PATH=(ADD PATH TO THE PX4 INSTALL LOCATION)
source $PX4_INSTALL_PATH/Tools/setup_gazebo.bash $PX4_INSTALL_PATH $PX4_INSTALL_PATH/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PX4_INSTALL_PATH
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PX4_INSTALL_PATH/Tools/sitl_gazebo
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins
```

For different version of PX4, location of the above files may be different. This guide is for PX4 firmware version 1.13. If you wish to use newer or older version of PX4, please change the export and source paths.

## Running
Launch PX4 and Gazebo
```
roslaunch px4 mavros_posix_sitl.launch
```
Launch the VIO utilities package
```
rosrun read_topic vio_transform
```
Launch Flightmare environment
```
roslaunch flightros rotors_gazebo.launch
```
Launch VINS-FUSION
``` 
rosrun vins vins_node $FLIGHTMARE_WS_PATH/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml
```
Launch MSF
```
roslaunch msf_updates viconpos_sensor.launch
```
Launch planner package
```
roslaunch ewok_optimization optimization_point.launch
```
Launch control package
```
roslaunch offboard planner.launch simulation:=true
```
choose 2 then 3 to enter planner flight mode

## Flight scenarios

**Control only**

Only use offboard control package to control the UAV in Flightmare, using odometry from mavros, input setpoints in enu flight mode

Config the control package to receive mavros local position topic as input odometry

**Control + Planning**

Use ewok planner to build map from depth camera and generate flight path, use offboard control with planning mode to follow the path from ewok

Config the planning and control package to receive mavros local position topic as input odometry

**Odometry only**

Use only offboard control with ENU flight mode or ewok planner + control to control the UAV, turn on VINS-Fusion package to check if VIO match with flight path

Config the planning and control package to receive mavros local position topic as input odometry. VIO package run independently

**Fly using odometry**

Use the output from VINS-Fusion to feed into ewok planner and offboard control to fly using odometry

Config the planning and control package to receive output topic of VIO package as input odometry.

**Fly using odometry with MSF**

Use the output from VINS-Fusion to feed into MSF to fuse with IMU data, then use output of MSF to fly with planning and control package

Config the MSF package to receive output of VIO package and IMU topic
Config the planning and control package to receive output topic of MSF package as input odometry.

## Instruction to config each package

**MSF**

- file launch in ethzasl_msf/msf_updates/launch/viconpos_sensor.launch show the remap for the pose input and imu input of the multi sensor fusion package

- /vio_odo_posestamped is output of odometry package change to posestamped format by VIO_utilities package

**Ewok optimization**

file ewok_optimization/launch/optimization_point.launch 
- have preset trajectory
- if change trajectory also need to change number_of_target

file ewok_optimization/example/spline_optimization_example3.cpp
- fx,fy,cx,cy depend on output image from Flightmare
- custom transform from /map to /base_link is created here
	- transform from /world to /map is in launch file
	- transform from /base_link to /camera_link is set in flightmare (as it output image)
	- to create a full tf tree from world to map to base_link to /camera_link to transform esdf map from /camera_link back to /world
- depth_image_sub_ sub to /depth topic from flightmare
- current_pose_sub sub to current pose to check for position of the drone to start output planning point. 
    - currently set to /msf_core/pose_after_update, set to /vio_odo_posestamped if you dont run msf
- to run without odometry
	- set current_pose_sub to /mavros/local_position/pose

**Flightmare**
file flightros/src/pilot/flight_pilot.cpp
- B_r_BC and B_r_BC2 set position of stereo camera (cam 1 double as depth camera)
- set POV, Width and Height of output image
- sub_state_est_ sub to /mavros/local_position/pose and each received pose render 2 RGB and 1 Depth image
- depth image have frame_id as /camera_link to use with ewok

**Control**

planner.launch
- have start and endpoint of trajectory set in planner, start and endpoint must be different otherwise the controller will detect start point is same is endpoint and land imediately
- change PID_kp to change accelleration to max speed (set in offboard_lib.cpp)

offboard_lib.cpp
- odom_sub_ to current pose, currently sub to /msf_core/odometry (output of msf), change to /mavros/local_position/odom to fly using groundtruth or /vio_odo to fly using output of odometry
- important function is the cal_vel function that calculate velocity from input setpoint and current position

**VINS-FUSION**

file config/euroc/euroc_stereo_imu_config.yaml is used when running the package
- config parameter have explaination comment from author

file euroc_stereo_imu_config.yaml, cam0_pinhole.yaml and cam1_pinhole.yaml need to match the image resolution

**VIO_utilities**

- take groundtruth and publish different ros message formats
- transform output of visual odometry from /vio_frame to /world frame and publish in diffrent formats

## Original ROS package

```
https://github.com/ethz-asl/ethzasl_msf

https://github.com/VladyslavUsenko/ewok

https://github.com/HKUST-Aerial-Robotics/VINS-Fusion.git
```
