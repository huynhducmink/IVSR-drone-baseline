# IVSR drone baseline

## A full autonomous drone navigation system with photo-realistic simulation environment

### Introduction



### Instalation
The simulation set is configured and verified to work on Ubuntu 20.04

Every ros package can be installed into 

**Install ROS Noetic**

http://wiki.ros.org/noetic/Installation/Ubuntu

**Install PX4 autopilot**

https://docs.px4.io/main/en/dev_setup/building_px4.html

**Change parameter setting in QGroundControl:**

- Download QGroundControl from http://qgroundcontrol.com/
- Start PX4
```
roslaunch px4 mavros_posix_sitl.launch
```
- Find parameter COM_RCL_EXCEPT in vehicle setting and set to 4 (Offboard mode)

**Install Flightmare**

- https://flightmare.readthedocs.io/en/latest/getting_started/quick_start.html#quick-start
- After finish installing Flightmare, replace the contents in the flightros folder with the contents in the following repository
```
git clone https://github.com/huynhducmink/flightros_for_PX4-Gazebo.git
```

**Install VINS-FUSION**

Eigen and Ceres don't need to install in ROS workspace, they will be installed to root

- Eigen 3.4.0 from https://eigen.tuxfamily.org/index.php?title=Main_Page
- Ceres-solver 2.1.0 from http://ceres-solver.org/installation.html (do not install libeigen3-dev, use Eigen version above)
```
git clone https://github.com/huynhducmink/VINS-Fusion
```

**Install Ewok**
```
sudo apt-get install git python python3-matplotlib python3-numpy libeigen3-dev libgoogle-glog-dev libatlas-base-dev libsuitesparse-dev protobuf-compiler libnlopt-dev libnlopt-cxx-dev ros-noetic-octomap ros-noetic-octomap-ros ros-noetic-octomap-msgs ros-noetic-tf-conversions ros-noetic-eigen-conversions

sudo apt install ros-noetic-sophus

git clone https://github.com/huynhducmink/ewok_optimization_DN.git
```
