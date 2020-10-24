# ramon_slam3d

Dependencies
------------
* [*ROS (FULL)*](http://wiki.ros.org/ROS/Installation)
* [*ramon_msgs*](https://gitlab.frba.utn.edu.ar/utnbadrone/utnbadrone)
* [*eigen3*](http://eigen.tuxfamily.org/index.php?title=Main_Page)
* [*PCL*](https://pointclouds.org/)
* [*pcl_ros*](http://wiki.ros.org/pcl_ros)
* [*openni2_launch*](http://wiki.ros.org/openni2_launch)

--------------
## Steps

### Install ROS and create the catkin workspace in, eg., ~/catkin_ws
Follow the ROS tutorials

### Install Eigen3 
    sudo apt-get install libeigen3-dev

### Create symlinks based on Eigen3 installation, eg, /usr/include
    cd /usr/include
    sudo ln -sf eigen3/Eigen Eigen
    sudo ln -sf eigen3/unsupported unsupported
    
### Install PCL library
    $ sudo apt install libpcl-dev

### Install pcl_ros
    $ sudo apt install ros-$ROS_DISTRO-pcl-ros

### Install Openni2_launch
    $ sudo apt-get install ros-$ROS_DISTRO-openni2-launch 

### Download sample bagfile and extract it in ramon_simulation/bagfiles/rosbot
* [turtebot3_world](https://drive.google.com/file/d/1g-gjRaW8C2rkBwlpe2pA-rbfG6eYGbCE/view?usp=sharing)