/**
 * @file ramon_slam2d.h
 * @author Francisco A. Dominguez (dominguezfranciscoa@gmail.com)
 * @brief
 * @version 0.1
 * @date 2020-01-09
 *
 * @copyright Copyright (c) 2020
 *
 */

#ifndef _ramon_slam2d_h
#define _ramon_slam2d_h

#include <algorithm>
#include <vector>
#include <boost/thread.hpp>
#include <eigen3/Eigen/Dense>

#include "ros/ros.h"
#include "ros/console.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point32.h"
#include "nav_msgs/OccupancyGrid.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"

#define MAP_IDX(map_width, x, y) ((map_width) * (y) + (x))
#define DEBUG_MODE -5

class RamonSlam2D
{
public:
  /* Public functions */
  RamonSlam2D();
  RamonSlam2D(uint32_t M, uint32_t N, float res);
  ~RamonSlam2D();

  void bresenhamLineAlgorithm(float x0, float y0, float x1, float y1, std::vector<geometry_msgs::Point32>& point);
  float gaussianBlurIntegral(float a, float b, float c, float std);

  float getLogitFromProba(float value);
  float getProbaFromLogit(float value);

  Eigen::Matrix2Xf rotateAndTranslate2D(Eigen::Matrix2Xf matrix, float x, float y, float theta);

  void start();

private:
  /* Private variables */

  // Node handle
  ros::NodeHandle nh_;

  // Frame names
  std::string base_frame_;
  std::string laser_frame_;
  std::string imu_frame_;
  std::string map_frame_;

  // Lidar counter and maximum distance value
  uint32_t laser_count_;
  float rmax_;

  // Map standard deviations (x,y)
  float std_x_, std_y_;

  // Float map (not an occupancygrid one)
  Eigen::MatrixXf map_eig_;

  int debug_;

  // ROS Publishers
  ros::Publisher map_pub_;
  ros::Publisher map_meta_pub_;

  // ROS Subscribers
  ros::Subscriber laser_sub_;

  // Threads
  boost::thread* transform_thread_;
  float transform_publish_period_;

  // Data from points
  float point_free_, point_noinfo_, point_occupied_;
  float point_dis_threshold_;
  uint32_t min_adjacent_points_;

  // Width (m_), height(n_) and resolution (res_)
  uint32_t m_, n_;
  float res_;

  // Vehicle position (x,y) and angle
  float x_, y_, theta_;

  // Transform delay
  float tf_delay_;

  // Broadcaster transform
  tf::TransformBroadcaster tf_broadcaster_;

  tf::Transform map_to_laser_;

  /* Private functions */

  // https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
  void bresenhamLineHigh(float x0, float y0, float x1, float y1, std::vector<geometry_msgs::Point32>& point);
  void bresenhamLineLow(float x0, float y0, float x1, float y1, std::vector<geometry_msgs::Point32>& point);

  void getOccupancyLikelihood(Eigen::Matrix3f& likelihood, Eigen::Vector2f point);

  Eigen::Matrix2Xf getPointsFromScan(sensor_msgs::LaserScan scan);

  void getRigidBodyTransform(Eigen::Matrix2Xf scan_points, Eigen::Matrix2Xf& scan_out);

  void init();

  Eigen::MatrixXf inverseScanner(Eigen::Matrix2Xf scan_points);

  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

  void logitUpdate(Eigen::MatrixXf& m, uint32_t index_x, uint32_t index_y, Eigen::Vector2f point);

  void publishTransform(void);
};

#endif  // _ramon_slam2d_h