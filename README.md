# ramon_slam


Dependencies
------------
* [*ROS (FULL)*](http://wiki.ros.org/ROS/Installation)
* [*ramon_msgs*](https://gitlab.frba.utn.edu.ar/utnbadrone/utnbadrone)
* [*eigen3*](http://eigen.tuxfamily.org/index.php?title=Main_Page)
* [*PCL*](https://pointclouds.org/)
* [*pcl_ros*](http://wiki.ros.org/pcl_ros)
* [*openni2_launch*](http://wiki.ros.org/openni2_launch)
* [*teleop_twist_keyboard*](http://wiki.ros.org/teleop_twist_keyboard)

--------------
## Installation Steps

### Install ROS and create the catkin workspace in, eg., ~/catkin_ws
Follow the ROS tutorials

### Install Eigen3 
    $ sudo apt-get install libeigen3-dev

### Create symlinks based on Eigen3 installation, eg, /usr/include
    $ cd /usr/include
    $ sudo ln -sf eigen3/Eigen Eigen
    $ sudo ln -sf eigen3/unsupported unsupported
    
### Install PCL library
    $ sudo apt install libpcl-dev

### Install pcl_ros
    $ sudo apt install ros-$ROS_DISTRO-pcl-ros

### Install Openni2_launch
    $ sudo apt-get install ros-$ROS_DISTRO-openni2-launch 

### Install telelop_twist_keyboard
    $ sudo apt-get install ros-$ROS_DISTRO-teleop-twist-keyboard

### Clone repo and switch to develop branch
    $ cd ~/catkin_ws/src
    $ git clone https://gitlab.frba.utn.edu.ar/utnbadrone/utnbadrone.git
    $ cd utnbadrone
    $ git checkout develop

### Make package
    $ cd ~/catkin_ws
    $ catkin_make

-------
## How to run examples

After following installation steps...

### SLAM2D
    $ roslaunch ramon_slam2d ramon_slam2d_examples.launch

### SLAM3D

#### Download sample bagfile and extract it in ramon_simulation/bagfiles/rosbot
    $ roscd ramon_simulation/bagfiles/rosbot
    $ wget https://drive.google.com/file/d/1g-gjRaW8C2rkBwlpe2pA-rbfG6eYGbCE/view?usp=sharing
    $ unzip turtlebot3_world.zip
    $ rm turtlebot3_world.zip
#### Run record
    $ roslaunch ramon_slam3d ramon_slam3d_examples.launch

### Calibration
    $ roslaunch ramon_calibration imu_calibration.launch

### Simulation
    $ roslaunch ramon_simulation rosbot_complete.launch

--------
## Workarounds

### If Gazebo error: [REST.cc:205] Error in REST request
    $ nano ~/.ignition/fuel/config.yaml

and replace https://api.ignitionfuel.org by https://api.ignitionrobotics.org
