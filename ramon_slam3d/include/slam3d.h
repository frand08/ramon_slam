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
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/io/pcd_io.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

// https://docs.ros.org/api/octomap_ros/html/conversions_8cpp.html
#include <sensor_msgs/point_cloud2_iterator.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
// #include <octomap_msgs/Octomap.h>

#include "map3d_utils.h"

#define DEBUG_MODE -5

namespace ramon_slam3d
{
	/**
	* @class SLAM3D
	* @brief Clase para representar celdas.
	*/

  class SLAM3D : public Map3DUtils
  {
  public:
    /* Public functions */
    SLAM3D();
    SLAM3D(float octree_res);
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

    // Topic names
    std::string depth_points_topic_;
    std::string imu_topic_;
    std::string ground_truth_topic_;

    // IMU usage
    bool use_imu_;
    bool got_imu_data_;

    // Odom usage
    bool pub_odom_;

    int debug_;
  

    // Broadcaster transform
    tf::TransformBroadcaster tf_broadcaster_;

    tf::Transform map_to_pointcloud_;

    // PointCloud2 used
    PointCloudT::Ptr cloud2_map_;

    // Store previous cloud
    PointCloudT::Ptr cloud_prev_filtered_;
    PointCloudT::Ptr cloud_prev_keypoints_;
    PointCloudNormalT::Ptr cloud_prev_normals_;
    PointCloudFeatureT::Ptr cloud_prev_features_;

    // Octomap
    octomap::OcTree* octree_map_;
    octomap::Pointcloud octomap_;
    bool use_octomap_;

    // ROS Publishers
    ros::Publisher camera_odom_pub_;
    ros::Publisher real_odom_pub_;
    ros::Publisher odom_pub_;
    ros::Publisher map_pub_;

    // ROS Subscribers
    ros::Subscriber pointcloud_sub_;
    ros::Subscriber ground_truth_sub_;

    // Check if start() was called
    bool started_;

    // Ground truth
    bool publish_ground_truth_;
    bool got_first_ground_truth_;

    // Transformation matrix (icp transform is float)
    Eigen::Matrix4f trans_;
    /* Private functions */

    int getPointCloudPCL(const sensor_msgs::PointCloud2 &pc2, PointCloudT &cloud_out);
    void groundTruthCallback(const nav_msgs::Odometry::ConstPtr& ground_truth);

    void init(void);

    void inertialCallback(const sensor_msgs::Imu::ConstPtr& imu);

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& pc2);
    void pointCloudCallback2(const sensor_msgs::PointCloud2ConstPtr& pc2ptr);

    void publishOdometry(void);
    void publishTransform(void);
  };
};

#endif  // _ramon_slam3d_h