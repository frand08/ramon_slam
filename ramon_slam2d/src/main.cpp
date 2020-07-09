#include <ros/ros.h>

#include "slam2d.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ramon_slam2d");

  ROS_INFO("Por init el Slam 2D");
  // ramon_slam2d::SLAM2D slam(16,16,0.02);
  ramon_slam2d::SLAM2D slam(100,100,0.02);
  ROS_INFO("Inicializado");
  slam.start();
  ros::spin();

  return 0;
}