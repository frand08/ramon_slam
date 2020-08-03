/**
 * @file slam2d.h
 * @author Francisco A. Dominguez (dominguezfranciscoa@gmail.com)
 * @brief
 * @version 0.1
 * @date 2020-01-09
 *
 * @copyright Copyright (c) 2020
 *
 */

#ifndef _slam2d_h
#define _slam2d_h

#include <algorithm>
#include <vector>
#include <boost/thread.hpp>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <fstream>

#include "map_utils.h"

#define MAP_IDX(map_width, x, y) ((map_width) * (y) + (x))
#define DEBUG_MODE -5

// Struct for rigid body transform threads
typedef struct
{
  Eigen::Matrix2Xd scan_in;
  double x;
  double y;
  double theta;
  double sum_out;
  double x_out;
  double y_out;
  double theta_out;
  double res;
  double theta_factor;
  Eigen::Matrix2Xd scan_out;
  Eigen::MatrixXd map;
} rigid_t;

namespace ramon_slam2d
{
	/**
	* @class SLAM2D
	* @brief Clase para representar celdas.
	*/

  class SLAM2D : public MapUtils
  {
  public:
    /* Public functions */
    SLAM2D();
    SLAM2D(uint32_t M, uint32_t N, double res);
    ~SLAM2D();

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

    // Topic names
    std::string scan_topic_name_;
    std::string imu_topic_name_;

    // IMU usage
    bool use_imu_;
    bool got_imu_data_;

    // Lidar counter and maximum distance value
    uint32_t laser_count_;
    double rmax_;

    // Map standard deviations (x,y)
    Eigen::Vector2d std_dev_;

    // number of maps
    int maps_number_;
    int current_map_;
    // double map (not an occupancygrid one)
    Eigen::MatrixXd map_eig_high_;
    Eigen::MatrixXd map_eig_med_;
    Eigen::MatrixXd map_eig_low_;
    std::vector<Eigen::MatrixXd> map_eig_vec_;

    // Points that conforms a map
    Eigen::Matrix2Xd map_points_;

    // Occupancygrid Map
    nav_msgs::OccupancyGrid occmap_;

    int debug_;

    // Number of maps
    int map_count_;

    // ROS Publishers
    ros::Publisher map_pub_;
    ros::Publisher map_meta_pub_;

    // ROS Subscribers
    ros::Subscriber laser_sub_;
    ros::Subscriber imu_sub_;

    // Threads
    boost::thread* transform_thread_;
    double transform_publish_period_;

    // Mutex
    boost::mutex laser_processing_mutex_;

    // Data from points
    double point_dis_threshold_;
    uint32_t min_adjacent_points_;

    // Width (m_), height(n_) and resolution (res_)
    uint32_t m_, n_;
    
    Eigen::VectorXd res_vec_;
    Eigen::VectorXd res_multipliers_;

    // Vehicle position (x,y) and angle
    double x_, y_, theta_;
    int x_iteration_, y_iteration_, theta_iteration_;
    double imu_theta_;
    double laser_imu_theta_;

    // Transform delay
    double tf_delay_;

    // Broadcaster transform
    tf::TransformBroadcaster tf_broadcaster_;

    tf::Transform map_to_laser_;

    /* Private functions */

    int getMaximumLikelihoodTransform(int theta_index, rigid_t& rigid);


    int getPointsFromScan(sensor_msgs::LaserScan scan, Eigen::Matrix2Xd &points_out);

    void getRigidBodyTransform(const Eigen::Ref<const Eigen::Matrix2Xd> scan_points, Eigen::Matrix2Xd &scan_out);

    void init(double res);

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

    void inertialCallback(const sensor_msgs::Imu::ConstPtr& imu);
    
    void mapsUpdate(Eigen::Matrix2Xd scan_points);

    void publishTransform(void);
  };
};
#endif  // _ramon_slam2d_h