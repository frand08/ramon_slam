/**
 * @file slam3d.cpp
 * @author Francisco A. Dominguez (dominguezfranciscoa@gmail.com)
 * @brief
 * @version 0.1
 * @date 2020-08-05
 *
 * @copyright Copyright (c) 2020
 *
 */

#include "slam3d.h"
#include <pcl/filters/voxel_grid.h>

using namespace ramon_slam3d;

// #define STORE_DATA_ONLY
#define HOLZ_APPROX

/* Public Functions */

/**
 * @brief Construct a new SLAM3D::SLAM3D object
 *
 */
SLAM3D::SLAM3D() : debug_(DEBUG_MODE), 
                   nh_("~"), 
                   cloud2_map_(new PointCloudSourceT)
{
  this->init();
}

/**
 * @brief Destroy the SLAM3D::SLAM3D object
 *
 */
SLAM3D::~SLAM3D()
{

}

/**
 * @brief Callback function of the ground truth topic, to publish the real odometry as a path
 *
 * @param scanptr Pointer to ground truth data
 */
void SLAM3D::groundTruthCallback(const nav_msgs::Odometry::ConstPtr& ground_truth)
{
  static nav_msgs::Path real_path;
  nav_msgs::Odometry gt = *ground_truth;
  geometry_msgs::PoseStamped real_pose;
  real_pose.header.stamp = ros::Time::now();
  real_path.header.stamp = real_pose.header.stamp;
  real_pose.header.frame_id = "real_path";
  real_path.header.frame_id = "real_path";
  
  // If the ground truth is used, the first value is stored, so that it 
  // has the same starting point as the estimated pose of the algorithm.
  if(!got_first_ground_truth_)
  {
    got_first_ground_truth_ = true;
    Eigen::Quaternionf q;
    q.x() = gt.pose.pose.orientation.x;
    q.y() = gt.pose.pose.orientation.y;
    q.z() = gt.pose.pose.orientation.z;
    q.w() = gt.pose.pose.orientation.w;
    Eigen::Matrix3f rot = q.normalized().toRotationMatrix();
    Eigen::Vector3f pos;
    pos << gt.pose.pose.position.x,
           gt.pose.pose.position.y,
           gt.pose.pose.position.z;

    // trans_ << rot(0,0), rot(0,1), rot(0,2), gt.pose.pose.position.x,
    //           rot(1,0), rot(1,1), rot(1,2), gt.pose.pose.position.y,
    //           rot(2,0), rot(2,1), rot(2,2), gt.pose.pose.position.z,
    //           0, 0, 0, 1;

    std::cout << "First trans_\n" << trans_ << std::endl;
  }
  if(started_)
  {
    real_pose.pose.position.x = gt.pose.pose.position.x;
    real_pose.pose.position.y = gt.pose.pose.position.y;
    real_pose.pose.position.z = 0;
    real_pose.pose.orientation = gt.pose.pose.orientation;

    real_path.poses.push_back(real_pose);
    real_path_pub_.publish(real_path);
  }
}

/**
 * @brief IMU topic callback function
 *
 * @param scanptr Pointer to imu data
 */
void SLAM3D::inertialCallback(const sensor_msgs::Imu::ConstPtr& imuptr)
{
  tf::Quaternion q(imuptr->orientation.x, imuptr->orientation.y, imuptr->orientation.z, imuptr->orientation.w);
  tf::Matrix3x3 m(q);
  double pitch, roll, yaw;
  m.getRPY(roll, pitch, yaw);
  // \todo something 
  got_imu_data_ = true;
}

/**
 * @brief Initializes the private variables of the class
 *
 */
void SLAM3D::init(void)
{
  double std_dev;

  started_ = false;

  // Define SLAM frames in case they were not given by user
  if (!nh_.getParam("base_frame", base_frame_))
    base_frame_ = "base_link";
  if (!nh_.getParam("map_frame", map_frame_))
    map_frame_ = "map";
  if (!nh_.getParam("imu_frame", imu_frame_))
    imu_frame_ = "imu";
  if (!nh_.getParam("pcd_save_location", pcd_save_location_))
    pcd_save_enabled_ = false;
  else
    pcd_save_enabled_ = true;
  if (!nh_.getParam("depth_points_topic", depth_points_topic_))
    depth_points_topic_ = "/depth";
  if (!nh_.getParam("imu_topic", imu_topic_))
    use_imu_ = false;
  else
    use_imu_ = true;
  if (!nh_.getParam("ground_truth_topic", ground_truth_topic_))
    publish_ground_truth_ = false;
  else
    publish_ground_truth_ = true;
  if (!nh_.getParam("std_dev", std_dev))
    std_dev = 0.1;
  if (!nh_.getParam("extodometry_topic_name", extodometry_topic_name_))
    use_ext_odom_ = false;
  else
    use_ext_odom_ = true;

  ROS_INFO("Params:");
  std::cout << "base_frame: " << base_frame_ << std::endl;
  std::cout << "map_frame: " << map_frame_ << std::endl;
  std::cout << "imu_frame: " << imu_frame_ << std::endl;
  std::cout << "depth_points_topic: " << depth_points_topic_ << std::endl;

  if(use_imu_)
  {
    std::cout << "use_imu: true" << std::endl;
    std::cout << "imu_topic: " << imu_topic_ << std::endl;
  }
  else
    std::cout << "use_imu: false" << std::endl;
  if(publish_ground_truth_)
  {
    std::cout << "publish_ground_truth: true" << std::endl;
    std::cout << "ground_truth_topic: " << ground_truth_topic_ << std::endl;
  }
  else
    std::cout << "publish_ground_truth: false" << std::endl;

  if(use_ext_odom_)
  {
    std::cout << "use_ext_odom: true" << std::endl;
    std::cout << "ext_odom_topic_name: " << extodometry_topic_name_ << std::endl;    
  }
  else
    std::cout << "use_ext_odom: false" << std::endl;

  pointcloud_frame_ = "pointcloud_slam3d";
  trans_.setIdentity();


  got_first_ground_truth_ = false;
  got_imu_data_ = false;
  got_ext_odom_ = false;
}
/**
 * @brief Callback function of the PointCloud2 topic
 *
 */
void SLAM3D::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& pc2ptr)
{
  static int count = 0;
  static nav_msgs::Path path;
  static ros::Time last = ros::Time::now();
  ros::Time now = ros::Time::now();
  ros::Time tf_expiration;
  // Para calcular tiempos del algoritmo
  ros::WallTime start, end;

  Eigen::Matrix4d trans_aux;

  geometry_msgs::PoseStamped pose;
  nav_msgs::Odometry odom;

  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  PointCloudSourceT::Ptr cloud_reg_raw(new PointCloudSourceT);
  PointCloudSourceT::Ptr cloud_reg_filtered(new PointCloudSourceT);
  PointCloudSourceT::Ptr cloud_reg_filtered_aux(new PointCloudSourceT);

  static PointCloudSourceT::Ptr cloud_prev_filtered(new PointCloudSourceT);

  Eigen::Matrix4d transform;

  if(use_ext_odom_)
    trans_aux = ext_trans_;
  else
    trans_aux.setIdentity();

  count++;
  
  // Init pose and path frames
  pose.header.frame_id = "robot_path3d";
  path.header.frame_id = "robot_path3d";
  odom.header.frame_id = "odom3d";
  odom.child_frame_id = "base_link";

  // If ground truth is used, init the transform with that data
  if(publish_ground_truth_ && !got_first_ground_truth_)
  {
    ROS_WARN("Waiting for ground truth data...");
    return;
  }
  start = ros::WallTime::now();

  cloud_reg_raw->height = pc2ptr->height;
  cloud_reg_raw->width = pc2ptr->width;
  // Get PCL pointcloud with selected data type, removing NaNs
  this->getPointCloudPCL(*pc2ptr, *cloud_reg_raw);

  if(pcd_save_enabled_)
  {
    if(count > 20 && count <=35)
    {
      ROS_INFO("Saving cloud %d", count);
      pcl::io::savePCDFileASCII(pcd_save_location_ + "/cloud" + std::to_string(count) + ".pcd", *cloud_reg_raw);
    }
    return;
  }
  
  this->filterAndUpsample(cloud_reg_raw, *cloud_reg_filtered);
  if(!cloud_prev_filtered->points.size())
  {
    // Store input values in previous ones
    *cloud_prev_filtered = *cloud_reg_filtered;

    // Downsample map to publish
    pcl::VoxelGrid<PointSourceT> grid;
    const float leaf = 0.1f;
    grid.setLeafSize(leaf, leaf, leaf);
    grid.setInputCloud(cloud_prev_filtered);
    grid.filter(*cloud_reg_filtered_aux);

    // Store first cloud in map
    if(use_ext_odom_)
    {
      trans_ = trans_aux;
      last_trans_ = trans_aux;
    }
    else
      last_trans_ = trans_;

    pcl::transformPointCloud(*cloud_reg_filtered_aux, *cloud_reg_filtered_aux, last_trans_);
    *cloud2_map_ = *cloud_reg_filtered_aux;
    
    return;
  }
  
  pcl::transformPointCloud(*cloud_reg_filtered, *cloud_reg_filtered_aux, last_trans_);
  if(this->registrationPipelineHolz(cloud_reg_filtered_aux, cloud_prev_filtered, transform) < 0)
  {
    ROS_INFO("Ignored cloud");
    return;
  }
  pcl::console::print_highlight("Estimated transform of cloud: %ld\n", count);
  pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transform (0,0), transform (0,1), transform (0,2));
  pcl::console::print_info("R = | %6.3f %6.3f %6.3f | \n", transform (1,0), transform (1,1), transform (1,2));
  pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transform (2,0), transform (2,1), transform (2,2));
  pcl::console::print_info("\n");
  pcl::console::print_info("t = < %0.3f, %0.3f, %0.3f >\n", transform (0,3), transform (1,3), transform (2,3));
  pcl::console::print_info("\n");
  // transform *= last_trans_;
  pcl::transformPointCloud(*cloud_reg_filtered_aux, *cloud_prev_filtered, transform);
  last_trans_ = transform * last_trans_;
  // trans_ = last_trans_ * trans_;
  
  // Downsample map to publish
  pcl::VoxelGrid<PointSourceT> grid;
  const float leaf = 0.1f;
  grid.setLeafSize(leaf, leaf, leaf);
  grid.setInputCloud(cloud_prev_filtered);
  grid.filter(*cloud_reg_filtered_aux);    

  // Add points to map
  // *cloud2_map_ += *cloud_prev_filtered;
  *cloud2_map_ += *cloud_reg_filtered_aux;

  // Convert from PCL data to ROS message
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg (*cloud2_map_, output);
  output.header.frame_id = "map";
  
  Eigen::AngleAxisd transformedAngleAxis(Eigen::Matrix3d(transform.block(0,0,3,3)));
  Eigen::Vector3d angular_velocity;
  angular_velocity << transformedAngleAxis.angle() * transformedAngleAxis.axis();
  
  pose.header.stamp = now;
  pose.pose.position.x = last_trans_(0,3);
  pose.pose.position.y = last_trans_(1,3);
  pose.pose.position.z = last_trans_(2,3);
  Eigen::Quaterniond q(Eigen::Matrix3d(last_trans_.block(0,0,3,3)));
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();

  odom.header.stamp = now;
  odom.pose.pose.position.x = last_trans_(0,3);
  odom.pose.pose.position.y = last_trans_(1,3);
  odom.pose.pose.position.z = last_trans_(2,3);
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();
  odom.twist.twist.linear.x = transform(0,3) / ((now - last).toNSec() * 1e-9);
  odom.twist.twist.linear.y = transform(1,3) / ((now - last).toNSec() * 1e-9);
  odom.twist.twist.linear.z = transform(2,3) / ((now - last).toNSec() * 1e-9);
  odom.twist.twist.angular.x = angular_velocity(0) / ((now - last).toNSec() * 1e-9);
  odom.twist.twist.angular.y = angular_velocity(1) / ((now - last).toNSec() * 1e-9);
  odom.twist.twist.angular.z = angular_velocity(2) / ((now - last).toNSec() * 1e-9);

  path.header.stamp = now;
  path.poses.push_back(pose);

  path_pub_.publish(path);
  odom_pub_.publish(odom);
  map_pub_.publish(output);
  
  end = ros::WallTime::now();
  double execution_time = (end - start).toNSec() * 1e-6;
  ROS_INFO_STREAM_COND(debug_ < 0, "Exectution time (ms): " << execution_time);

  // Update time
  last = now;
}

/**
 * @brief Begins the 3DSLAM algorithm, advertising and subscribing to the needed topics
 *
 */
void SLAM3D::start(void)
{
  map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("map", 1);
  
  if(publish_ground_truth_)
  {
    ground_truth_sub_ = nh_.subscribe(ground_truth_topic_, 1, &SLAM3D::groundTruthCallback, this);
    real_path_pub_ = nh_.advertise<nav_msgs::Path>("real_path",1);
  }

  if(use_ext_odom_)
    extodom_sub_ = nh_.subscribe(extodometry_topic_name_, 1, &SLAM3D::extOdometryCallback, this);

  path_pub_ = nh_.advertise<nav_msgs::Path>("robot_path3d", 1);

  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("robot_odom3d",1);
  
  pointcloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(depth_points_topic_, 1, &SLAM3D::pointCloudCallback, this);
  started_ = true;
}


/**
 * @brief Callback function of the external odometry of the vehicle
 *
 */
void SLAM3D::extOdometryCallback(const nav_msgs::Odometry::ConstPtr& odomptr)
{
  nav_msgs::Odometry ext_odom = *odomptr;
  Eigen::Quaterniond aux;
  
  tf::quaternionMsgToEigen(ext_odom.pose.pose.orientation, aux);
  
  ext_trans_.block(0,0,3,3) = aux.normalized().toRotationMatrix();
  ext_trans_(0,3) = ext_odom.pose.pose.position.x;
  ext_trans_(1,3) = ext_odom.pose.pose.position.y;
  ext_trans_(2,3) = ext_odom.pose.pose.position.z;
  ext_trans_(3,0) = 0;
  ext_trans_(3,1) = 0;
  ext_trans_(3,2) = 0;
  ext_trans_(3,3) = 1;

  got_ext_odom_ = true;
}


int SLAM3D::getPointCloudPCL(const sensor_msgs::PointCloud2 &pc2, PointCloudSourceT &cloud_out)
{
  // Indices to remove NAN
  std::vector<int> indices;

  // PCL PointCloud2
  pcl::PCLPointCloud2 pcl_pc2;   

  // Convert from ROS message to PCL data
  pcl_conversions::toPCL(pc2, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, cloud_out);

  // Organized clouds are non-dense. The reason is that you need invalid points to "fill up" 
  // the point cloud for pixels that don't have a valid measurement: DO NOT REMOVE NAN POINTS
  //remove NAN points from the cloud
  // pcl::removeNaNFromPointCloud(cloud_out, cloud_out, indices);

  return 0;
}