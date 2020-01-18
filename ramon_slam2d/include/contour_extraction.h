#ifndef _contour_extraction_h
#define _contour_extraction_h

#include "geometry_msgs/Point32.h"

void contour_extraction(std::vector<std::vector<geometry_msgs::Point32> > &contour_array,
                        std::vector<geometry_msgs::Point32> &points32_aux,
                        float dis_threshold);
#endif