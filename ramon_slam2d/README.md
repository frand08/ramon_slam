# ramon_slam2d

Dependencies
------------
* [*ROS (FULL)*](http://wiki.ros.org/ROS/Installation)
* [*ramon_msgs*](https://gitlab.frba.utn.edu.ar/utnbadrone/utnbadrone)
* [*eigen3*](http://eigen.tuxfamily.org/index.php?title=Main_Page)

-------------
## Steps

### Install ROS and create the catkin workspace in, eg., ~/catkin_ws
Follow the ROS tutorials

### Install Eigen3 
    $ sudo apt-get install libeigen3-dev

### Create symlinks based on Eigen3 installation, eg, /usr/include
    $ cd /usr/include
    $ sudo ln -sf eigen3/Eigen Eigen
    $ sudo ln -sf eigen3/unsupported unsupported