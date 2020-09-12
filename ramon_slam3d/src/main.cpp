#include <ros/ros.h>

#include "slam3d.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ramon_slam3d");

  ROS_INFO("Por init el Slam 3D");
  ramon_slam3d::SLAM3D slam;
  ROS_INFO("Inicializado");
  slam.start();
  ros::spin();

  return 0;
}