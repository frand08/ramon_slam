_tkinter needed for matplotlibcpp. Install it in Debian based distros using 
sudo apt-get install python-tk

# rosbot simulation
- git clone https://github.com/husarion/rosbot_description.git

- if RLException: while processing (rosbot_description location)/launch/rosbot_gazebo.launch:
Invalid <param> tag: Cannot load command parameter [robot_description]: no such command [['/opt/ros/noetic/share/xacro/xacro.py', '(rosbot_description location)/urdf/rosbot.xacro']].

modify xacro.py with xacro

- sudo apt-get install ros-$ROS_DISTRO-teleop-twisted-keyboard