%% Leo el bag
clc;close all;clear;
% test1.bag tiene:
% -/f_scan: sensor_msgs/LaserScan
% -/imu_data: sensor_msgs/Imu 

% test2.bag tiene:
% -/f_scan: sensor_msgs/LaserScan
% -/imu_data: sensor_msgs/Imu 

% test3.bag tiene:
% -/horizontal_laser_2d: sensor_msgs/MultiEchoLaserScan
% -/imu: sensor_msgs/Imu 

% test4.bag tiene:
% -/b_scan       105 msgs    : sensor_msgs/LaserScan             
% -/clock       1067 msgs    : rosgraph_msgs/Clock               
% -/f_scan       105 msgs    : sensor_msgs/LaserScan             
% -/imu_data     416 msgs    : sensor_msgs/Imu                   
% -/landmark      51 msgs    : cartographer_ros_msgs/LandmarkList
% -/odom_enc     344 msgs    : nav_msgs/Odometry                 
% -/rosout         8 msgs    : rosgraph_msgs/Log

% test5.bag tiene:
% topics:      /f_scan      132 msgs    : sensor_msgs/LaserScan
%              /imu_data    519 msgs    : sensor_msgs/Imu      
%              /odom_enc    427 msgs    : nav_msgs/Odometry    
%              /tf          621 msgs    : tf2_msgs/TFMessage   
%              /tf_static     1 msg     : tf2_msgs/TFMessage

bag = rosbag('test5.bag');
msgs_cell = readMessages(bag);          %% Obtengo todos los mensajes
cant_msgs = length(msgs_cell);

imu_to_base_trans = bag.getTransform('base_link','imu_link');
lidar_to_base_trans = bag.getTransform('base_link','front_laser_link');


%%

tfTree = rostf;