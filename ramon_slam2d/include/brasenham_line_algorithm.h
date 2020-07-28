#ifndef _brasenham_line_algorithm_h
#define _brasenham_line_algorithm_h

#include <geometry_msgs/Point32.h>

void squares_line_low(float x0, float y0, float x1, float y1, 
                      std::vector<geometry_msgs::Point32> &point);

void squares_line_high(float x0, float y0, float x1, float y1, 
                       std::vector<geometry_msgs::Point32> &point);

void squares_line(float x0, float y0, float x1, float y1, 
                  std::vector<geometry_msgs::Point32> &point);

#endif