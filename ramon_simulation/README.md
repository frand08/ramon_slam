# ramon_simulation

Dependencies
------------
* [*ROS (FULL)*](http://wiki.ros.org/ROS/Installation)
* [*openni2_launch*](http://wiki.ros.org/openni2_launch)
* [*teleop_twist_keyboard*](http://wiki.ros.org/teleop_twist_keyboard)

--------------
## Steps

### Install ROS and create the catkin workspace in, eg., ~/catkin_ws
Follow the ROS tutorials

### Install Openni2_launch
    $ sudo apt-get install ros-$ROS_DISTRO-openni2-launch

### Install telelop_twist_keyboard
    $ sudo apt-get install ros-$ROS_DISTRO-teleop-twist-keyboard

----------
## Workarounds

### If Gazebo error: [REST.cc:205] Error in REST request
    $ nano ~/.ignition/fuel/config.yaml

and replace https://api.ignitionfuel.org by https://api.ignitionrobotics.org
