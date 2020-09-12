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

// #define USE_REAL_ODOM_ONLY
// #define STORE_PCLS

using namespace ramon_slam3d;


/* Public Functions */

/**
 * @brief Construct a new SLAM3D::SLAM3D object
 *
 */
SLAM3D::SLAM3D() : debug_(DEBUG_MODE), 
                   nh_("~"), 
                   cloud2_map_(new PointCloudT),
                   octree_map_(new octomap::OcTree(0.1f)),
                   cloud_prev_filtered_(new PointCloudT),
                   cloud_prev_keypoints_(new PointCloudT),
                   cloud_prev_normals_(new PointCloudNormalT),
                   cloud_prev_features_(new PointCloudFeatureT)
{
  this->init();
}

/**
 * @brief Construct a new SLAM3D::SLAM3D object
 *
 */
SLAM3D::SLAM3D(float octree_res) : debug_(DEBUG_MODE), 
                                   nh_("~"), 
                                   cloud2_map_(new PointCloudT),
                                   octree_map_(new octomap::OcTree(octree_res)),
                                   cloud_prev_filtered_(new PointCloudT),
                                   cloud_prev_keypoints_(new PointCloudT),
                                   cloud_prev_normals_(new PointCloudNormalT),
                                   cloud_prev_features_(new PointCloudFeatureT)
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
  static nav_msgs::Path real_odom;
  nav_msgs::Odometry gt = *ground_truth;
  geometry_msgs::PoseStamped real_pose;
  real_pose.header.stamp = ros::Time::now();
  real_odom.header.stamp = real_pose.header.stamp;
  real_pose.header.frame_id = "real_odom";
  real_odom.header.frame_id = "real_odom";
  
  // If the ground truth is used, the first value is stored, so that it 
  // has the same starting point as the estimated pose of the algorithm.
#ifndef USE_REAL_ODOM_ONLY
  if(!got_first_ground_truth_)
#endif

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
#ifndef USE_REAL_ODOM_ONLY
    std::cout << "First trans_\n" << trans_ << std::endl;
#endif
  }
  if(started_)
  {
    real_pose.pose.position.x = gt.pose.pose.position.x;
    real_pose.pose.position.y = gt.pose.pose.position.y;
    real_pose.pose.position.z = 0;
    real_pose.pose.orientation = gt.pose.pose.orientation;

    real_odom.poses.push_back(real_pose);
    real_odom_pub_.publish(real_odom);
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
  if (!nh_.getParam("use_octomap", use_octomap_))
    use_octomap_ = false;

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

  pointcloud_frame_ = "pointcloud_slam3d";
  trans_.setIdentity();

  got_first_ground_truth_ = false;
  got_imu_data_ = false;


}
/**
 * @brief Callback function of the PointCloud2 topic
 *
 */
void SLAM3D::pointCloudCallback2(const sensor_msgs::PointCloud2ConstPtr& pc2ptr)
{
  static int count = 0;
  static nav_msgs::Path odom;
  geometry_msgs::PoseStamped pose;
  ros::Time now = ros::Time::now();
  count++;
  ros::Time tf_expiration;
  // Para calcular tiempos del algoritmo
  ros::WallTime start, end;

  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  PointCloudT::Ptr cloud_reg_raw(new PointCloudT);
  PointCloudT::Ptr cloud_reg_filtered(new PointCloudT);
  PointCloudT::Ptr cloud_reg_keypoints(new PointCloudT);
  PointCloudNormalT::Ptr cloud_reg_normals(new PointCloudNormalT);
  PointCloudFeatureT::Ptr cloud_reg_features(new PointCloudFeatureT);

  static PointCloudT::Ptr cloud_prev_filtered(new PointCloudT);
  PointCloudT::Ptr cloud_prev_keypoints(new PointCloudT);
  PointCloudNormalT::Ptr cloud_prev_normals(new PointCloudNormalT);
  PointCloudFeatureT::Ptr cloud_prev_features(new PointCloudFeatureT);

  // Init pose and odom frames
  pose.header.frame_id = "robot_odom";
  odom.header.frame_id = "robot_odom";
  ROS_INFO("Test");

  // If ground truth is used, init the transform with that data
  if(publish_ground_truth_ && !got_first_ground_truth_)
  {
    ROS_WARN("Waiting for ground truth data...");
    return;
  }
  start = ros::WallTime::now();

  // Get PCL pointcloud with selected data type, removing NaNs
  this->getPointCloudPCL(*pc2ptr, *cloud_reg_raw);

  // Downsample input cloud
  this->downsample(cloud_reg_raw, *cloud_reg_filtered);

  // Transform input value based on previous transform
  pcl::transformPointCloud(*cloud_reg_filtered, *cloud_reg_filtered, trans_);

  // If first time, store values only, otherwise, search best transform
  if(!cloud_prev_filtered->points.size())
  {
    // Store input values in previous ones
    *cloud_prev_filtered = *cloud_reg_filtered;

    // Store first cloud in map
    *cloud2_map_ = *cloud_prev_filtered;
    
    return;
  }

  // Estimate keypoints
  this->estimateSIFTKeypoints(cloud_reg_filtered, *cloud_reg_keypoints);
  this->estimateSIFTKeypoints(cloud_prev_filtered, *cloud_prev_keypoints);

  // Estimate normals
  this->estimateNormalsOMP(cloud_reg_filtered, *cloud_reg_normals);
  this->estimateNormalsOMP(cloud_prev_filtered, *cloud_prev_normals);

  // Estimate features
  this->estimateFPFH(cloud_reg_filtered, cloud_reg_normals, cloud_reg_keypoints, *cloud_reg_features);
  this->estimateFPFH(cloud_prev_filtered, cloud_prev_normals, cloud_prev_keypoints, *cloud_prev_features);

  // // Find correspondences between keypoints in FPFH space
  // pcl::CorrespondencesPtr all_correspondences (new pcl::Correspondences), 
  //                         good_correspondences (new pcl::Correspondences);

  // this->findCorrespondences(cloud_reg_filtered, cloud_prev_filtered_, *all_correspondences);

  // // Reject correspondences based on their XYZ distance
  // this->rejectBadCorrespondences(all_correspondences, 
  //                                cloud_reg_filtered, 
  //                                cloud_prev_filtered_, 
  //                                *good_correspondences);

  // Compute initial transform using RANSAC
  Eigen::Matrix4f ransac_tf;
  int ransac_ret = this->ransac(cloud_reg_keypoints, 
                                cloud_reg_features,
                                cloud_prev_keypoints, 
                                cloud_prev_features,
                                ransac_tf);
  
  if(ransac_ret < 0)
  {
    ROS_ERROR("RANSAC ERROR");
    return;
  }
  
  // Transform input pointcloud based on ransac transform
  pcl::transformPointCloud(*cloud_reg_filtered, *cloud_reg_filtered, ransac_tf);

  // Compute ICP transform based on RANSAC
  Eigen::Matrix4f icp_tf;

  int icp_ret = this->icp(cloud_reg_filtered, cloud_prev_filtered, icp_tf);

  if(icp_ret < 0)
  {
    ROS_ERROR("ICP ERROR");
    return;
  }

  // Final transform
  Eigen::Matrix4f combined_tf;
  combined_tf = icp_tf * ransac_tf;

  // Save data based on estimated transforms
  pcl::transformPointCloud(*cloud_reg_filtered, *cloud_prev_filtered, icp_tf);

  // Update transform
  trans_ = combined_tf * trans_;

  // Add points to map
  *cloud2_map_ += *cloud_prev_filtered;

  // Convert from PCL data to ROS message
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg (*cloud2_map_, output);
  output.header.frame_id = "map";

  map_pub_.publish(output);

  odom.header.stamp = now;
  pose.header.stamp = now;
  pose.pose.position.x = trans_(0,3);
  pose.pose.position.y = trans_(1,3);
  pose.pose.position.z = trans_(2,3);
  Eigen::Quaternionf q(Eigen::Matrix3f(trans_.block(0,0,3,3)));
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();
  odom.poses.push_back(pose);
  odom_pub_.publish(odom);
  end = ros::WallTime::now();
  double execution_time = (end - start).toNSec() * 1e-6;
  ROS_INFO_STREAM_COND(debug_ < 0, "Exectution time (ms): " << execution_time);
}


/**
 * @brief Callback function of the PointCloud2 topic
 *
 */
void SLAM3D::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& pc2ptr)
{
  static int count = 0;
  static nav_msgs::Path odom;
  geometry_msgs::PoseStamped pose;
  ros::Time now = ros::Time::now();
  count++;
  ros::Time tf_expiration;
  // Para calcular tiempos del algoritmo
  ros::WallTime start, end;
  static PointCloudT::Ptr cloud2_prev(
                          new PointCloudT
                          );
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  PointCloudT::Ptr cloud2_in(
                          new PointCloudT
                          );
  // Filtered pointcloud
  PointCloudT::Ptr cloud2_filtered(
                          new PointCloudT
                          );
  // Transformed input PointCloud
  PointCloudT::Ptr cloud2_tr(
                          new PointCloudT
                          );

  pcl::PCLPointCloud2 pcl_pc2;                        
  // Create the filtering object
  pcl::VoxelGrid<PointT> sor;

  sensor_msgs::PointCloud2 output;
  // Indices to remove NAN
  std::vector<int> indices;


  pose.header.frame_id = "robot_odom";
  odom.header.frame_id = "robot_odom";
  ROS_INFO("count: %d",count);
  // If ground truth is used, init the transform with that data
  if(publish_ground_truth_ && !got_first_ground_truth_)
  {
    ROS_WARN("Waiting for ground truth data...");
    return;
  }
  start = ros::WallTime::now();

  // Convert from ROS message to PCL data
  // pcl::fromROSMsg ( *pc2ptr, *cloud2_in );
  pcl_conversions::toPCL(*pc2ptr, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2,*cloud2_in);

  //remove NAN points from the cloud
  pcl::removeNaNFromPointCloud(*cloud2_in, *cloud2_in, indices);

#ifdef STORE_PCLS
  if(count == 1)
  {
    pcl::io::savePCDFileASCII ("/home/fdominguez/Downloads/Tests_PCL/Inputs2/test_pcd_raw_1.pcd", *cloud2_in);
    std::cout << "trans_1:\n" << trans_ << std::endl;
  }
  if(count == 10)
  {
    pcl::io::savePCDFileASCII ("/home/fdominguez/Downloads/Tests_PCL/Inputs2/test_pcd_raw_10.pcd", *cloud2_in);
    std::cout << "trans_10:\n" << trans_ << std::endl;
  }
  if(count == 20)
  {
    pcl::io::savePCDFileASCII ("/home/fdominguez/Downloads/Tests_PCL/Inputs2/test_pcd_raw_20.pcd", *cloud2_in);
    std::cout << "trans_20:\n" << trans_ << std::endl;
  }
  if(count == 30)
  {
    pcl::io::savePCDFileASCII ("/home/fdominguez/Downloads/Tests_PCL/Inputs2/test_pcd_raw_30.pcd", *cloud2_in);
    std::cout << "trans_30:\n" << trans_ << std::endl;
  }
  if(count == 80)
  {
    pcl::io::savePCDFileASCII ("/home/fdominguez/Downloads/Tests_PCL/Inputs2/test_pcd_raw_80.pcd", *cloud2_in);
    std::cout << "trans_1:\n" << trans_ << std::endl;
  }
#endif

  // Set filter params
  sor.setInputCloud (cloud2_in);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  // Filter input PointCloud
  sor.filter (*cloud2_filtered);

  // Store the cloud the first time
  if(!cloud2_prev->points.size())
  {
    pcl::transformPointCloud (*cloud2_filtered, *cloud2_prev, trans_);
    // *cloud2_prev = *cloud2_filtered;
    *cloud2_map_ = *cloud2_prev;
    return;
  }
#ifndef USE_REAL_ODOM_ONLY
  else
  {
    // Transform based on previous data
    pcl::transformPointCloud (*cloud2_filtered, *cloud2_tr, trans_); 
    // *cloud2_tr = *cloud2_filtered;
  }
  
  pcl::IterativeClosestPoint<PointT, PointT> icp;

  // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
  icp.setMaxCorrespondenceDistance (0.05);
  // Set the maximum number of iterations (criterion 1)
  icp.setMaximumIterations (10);
  // Set the transformation epsilon (criterion 2)
  icp.setTransformationEpsilon (1e-8);
  // Set the euclidean distance difference epsilon (criterion 3)
  icp.setEuclideanFitnessEpsilon (1);
  // icp.setRANSACOutlierRejectionThreshold (1.5);

  // Set input cloud and previous pointcloud
  icp.setInputSource(cloud2_prev);
  icp.setInputTarget(cloud2_tr);
  // icp.setInputCloud(cloud2_prev);
  // icp.setInputTarget(cloud2_in);

  // Perform ICP
  PointCloudT Final;
  icp.align(Final);


  // pcl::transformPointCloud (Final, Final, trans_);
  // Update transform
  trans_ = icp.getFinalTransformation() * trans_;
  
  // Convert Eigen::Matrix4f to tf::Transform
  tf::Vector3 origin;
  origin.setValue(static_cast<double>(trans_(0,3)),
                  static_cast<double>(trans_(1,3)),
                  static_cast<double>(trans_(2,3)));
  tf::Matrix3x3 tf3d;
  tf3d.setValue(static_cast<double>(trans_(0,0)), static_cast<double>(trans_(0,1)), static_cast<double>(trans_(0,2)), 
                static_cast<double>(trans_(1,0)), static_cast<double>(trans_(1,1)), static_cast<double>(trans_(1,2)), 
                static_cast<double>(trans_(2,0)), static_cast<double>(trans_(2,1)), static_cast<double>(trans_(2,2)));

  tf::Quaternion tfqt;
  tf3d.getRotation(tfqt);
  
  // map_to_pointcloud_ = tf::Transform(tfqt, origin);
  map_to_pointcloud_.setOrigin(origin);
  map_to_pointcloud_.setRotation(tfqt);

  if(icp.hasConverged())
#endif
  {
#ifndef USE_REAL_ODOM_ONLY
    // If octomap_server is used, send transform and the PointCloud2 input, otherwise send PointCloud2 map
    if(use_octomap_)
    {
      tf_expiration = ros::Time::now() + ros::Duration(0.1);

      // Convert from PCL data to ROS message
      pcl::toROSMsg (*cloud2_tr, output);
      // pcl::toROSMsg (Final, output);
      output.header.frame_id = pointcloud_frame_;

      // Publish transform
      tf_broadcaster_.sendTransform(tf::StampedTransform(map_to_pointcloud_, tf_expiration, map_frame_, pointcloud_frame_));
    }
    else
#endif
    {
#ifndef USE_REAL_ODOM_ONLY

      *cloud2_prev = Final;
      *cloud2_map_ += *cloud2_prev;
#else
      pcl::transformPointCloud (*cloud2_filtered, *cloud2_tr, trans_);
      // pcl::transformPointCloud (*cloud2_map_, *cloud2_map_, trans_);
      *cloud2_map_ += *cloud2_tr;
#endif
      // Convert from PCL data to ROS message
      pcl::toROSMsg (*cloud2_map_, output);
      output.header.frame_id = "map";
    }
    map_pub_.publish(output);
  }
#ifndef USE_REAL_ODOM_ONLY
  else
  {
    ROS_ERROR("No convergio");
  }
#endif
  odom.header.stamp = now;
  pose.header.stamp = now;
  pose.pose.position.x = trans_(0,3);
  pose.pose.position.y = trans_(1,3);
  pose.pose.position.z = trans_(2,3);
  Eigen::Quaternionf q(Eigen::Matrix3f(trans_.block(0,0,3,3)));
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();
  odom.poses.push_back(pose);
  odom_pub_.publish(odom);
  end = ros::WallTime::now();
  double execution_time = (end - start).toNSec() * 1e-6;
#ifndef USE_REAL_ODOM_ONLY
  ROS_INFO_STREAM_COND(debug_ < 0, "ICP Fitness score: " << icp.getFitnessScore());
#endif
  ROS_INFO_STREAM_COND(debug_ < 0, "Exectution time (ms): " << execution_time);
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
    real_odom_pub_ = nh_.advertise<nav_msgs::Path>("real_odom",1);
  }

  odom_pub_ = nh_.advertise<nav_msgs::Path>("robot_odom", 1);

  // pointcloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(depth_points_topic_, 1, &SLAM3D::pointCloudCallback, this);
  // ROS_INFO("Callback");
  pointcloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(depth_points_topic_, 1, &SLAM3D::pointCloudCallback2, this);
  ROS_INFO("Callback 2");
  started_ = true;
}




int SLAM3D::getPointCloudPCL(const sensor_msgs::PointCloud2 &pc2, PointCloudT &cloud_out)
{
  // Indices to remove NAN
  std::vector<int> indices;

  // PCL PointCloud2
  pcl::PCLPointCloud2 pcl_pc2;   

  // Convert from ROS message to PCL data
  pcl_conversions::toPCL(pc2, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, cloud_out);

  //remove NAN points from the cloud
  pcl::removeNaNFromPointCloud(cloud_out, cloud_out, indices);

  return 0;
}