#include <ros/ros.h>

#include "ramon_slam2d.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ramon_slam2d");

  ROS_INFO("Por init el Slam 2D");
  RamonSlam2D slam(16,16,0.01);
  ROS_INFO("Inicializado");
  slam.start();
  ros::spin();

  return 0;
}