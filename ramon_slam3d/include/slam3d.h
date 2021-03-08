/**
 * @file slam3d.h
 * @author Francisco A. Dominguez (dominguezfranciscoa@gmail.com)
 * @brief
 * @version 0.1
 * @date 2020-08-05
 *
 * @copyright Copyright (c) 2020
 *
 */

#ifndef _slam3d_h
#define _slam3d_h

#include <algorithm>
#include <vector>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <fstream>

#include <pcl_ros/point_cloud.h>

#include <pcl/io/pcd_io.h>

#include <eigen_conversions/eigen_msg.h>

#include "map3d_utils.h"

#define DEBUG_MODE -5


typedef pcl::PointXYZRGB PointSourceT;
typedef pcl::PointCloud<PointSourceT> PointCloudSourceT;
typedef pcl::PointXYZRGBNormal PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace ramon_slam3d
{
	/**
	* @class SLAM3D
	* @brief Clase para representar celdas.
	*/

  class SLAM3D : public Map3DUtils<PointSourceT, PointT>
  {
  public:
    /* Public functions */
    SLAM3D();
    ~SLAM3D();

    void start();

  private:
    /* Private variables */

    // Node handle
    ros::NodeHandle nh_;

    // Frame names
    std::string base_frame_;
    std::string depth_frame_;
    std::string imu_frame_;
    std::string map_frame_;
    std::string pointcloud_frame_;
    std::string pcd_save_location_;
    std::string extodometry_topic_name_;

    // Topic names
    std::string depth_points_topic_;
    std::string imu_topic_;
    std::string ground_truth_topic_;

    // IMU usage
    bool use_imu_;
    bool got_imu_data_;

    // Path usage
    bool pub_path_;

    // External odom usage
    bool use_ext_odom_;
    bool got_ext_odom_;

    
    int debug_;
  
    // Save in pcd data or not
    bool pcd_save_enabled_;
  
    // Broadcaster transform
    tf::TransformBroadcaster tf_broadcaster_;

    tf::Transform map_to_pointcloud_;

    // PointCloud2 used
    PointCloudSourceT::Ptr cloud2_map_;

    // ROS Publishers
    ros::Publisher real_path_pub_;
    ros::Publisher path_pub_;
    ros::Publisher map_pub_;
    ros::Publisher odom_pub_;

    // ROS Subscribers
    ros::Subscriber pointcloud_sub_;
    ros::Subscriber ground_truth_sub_;
    ros::Subscriber extodom_sub_;

    // Check if start() was called
    bool started_;

    // Ground truth
    bool publish_ground_truth_;
    bool got_first_ground_truth_;

    // Transformation matrix (icp transform is float)
    Eigen::Matrix4d trans_;
    Eigen::Matrix4d last_trans_;
    Eigen::Matrix4d ext_trans_;
    /* Private functions */

    int getPointCloudPCL(const sensor_msgs::PointCloud2 &pc2, PointCloudSourceT &cloud_out);
    void groundTruthCallback(const nav_msgs::Odometry::ConstPtr& ground_truth);

    void init(void);

    void inertialCallback(const sensor_msgs::Imu::ConstPtr& imu);

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& pc2);

    void extOdometryCallback(const nav_msgs::Odometry::ConstPtr& odomptr);

    void publishOdometry(void);
    void publishTransform(void);
  };
};

#endif  // _ramon_slam3d_h