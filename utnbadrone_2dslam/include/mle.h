#ifndef _mle_h
#define _mle_h

#include <cmath>
#include "geometry_msgs/Point32.h"
#include <ros/ros.h>

float get_likelihood(float x, float m, float s);
void get_occupancy_likelihood(std::vector<int8_t> &likelihood, geometry_msgs::Point32 point, float resolution, float std_x, float std_y);
float gaussian_blur_integral(float a, float b, float c, float std);

#endif