/**
 * @file slam2d.cpp
 * @author Francisco A. Dominguez (dominguezfranciscoa@gmail.com)
 * @brief
 * @version 0.1
 * @date 2020-01-09
 *
 * @copyright Copyright (c) 2020
 *
 */

#include "slam2d.h"

using namespace ramon_slam2d;

/* Public Functions */

/**
 * @brief Construct a new SLAM2D::SLAM2D object
 *
 */
SLAM2D::SLAM2D() : transform_thread_(NULL), debug_(DEBUG_MODE), nh_("~")
{
  m_ = 50;
  n_ = 50;
  double res = 0.02;

  this->init(res);
}

/**
 * @brief Construct a new SLAM2D::SLAM2D object
 *
 * @param M
 * @param N
 * @param res
 */
SLAM2D::SLAM2D(uint32_t M, uint32_t N, double res) : transform_thread_(NULL), debug_(DEBUG_MODE), nh_("~")
{
  m_ = M;
  n_ = N;

  this->init(res);
}

/**
 * @brief Destroy the SLAM2D::SLAM2D object
 *
 */
SLAM2D::~SLAM2D()
{
  if (transform_thread_)
  {
    transform_thread_->join();
    delete transform_thread_;
  }
}

/**
 * @brief Begins the 2DSlam algorithm, advertising and subscribing to the needed topics
 *
 */
void SLAM2D::start()
{
  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1);
  laser_sub_ = nh_.subscribe(scan_topic_name_, 1, &SLAM2D::laserCallback, this);  // check the last param
  if(use_imu_)
    imu_sub_ = nh_.subscribe(imu_topic_name_, 1, &SLAM2D::inertialCallback, this);
  
  if(use_ext_odometry_)
    extodom_sub_ = nh_.subscribe(extodometry_topic_name_, 1, &SLAM2D::extOdometryCallback, this);
  
  // TF thread
  if(publish_map_to_laser_)
    transform_thread_ = new boost::thread(boost::bind(&SLAM2D::publishTransform, this));

  path_pub_ = nh_.advertise<nav_msgs::Path>("robot_path2d", 1);

  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("robot_odom2d",1);
  
  if(publish_ground_truth_)
  {
    ground_truth_sub_ = nh_.subscribe(ground_truth_topic_name_, 1, &SLAM2D::groundTruthCallback, this);
    real_path_pub_ = nh_.advertise<nav_msgs::Path>("real_path",1);
  }
  started_ = true;
}

/* Private functions */

/**
 * @brief Gets the maximum likelihood transform for a given theta index
 *
 * @param theta_index index of angle
 * @param rigid Information about the input scan, offsets and output scans (reference)
 * @return int zero value
 */
int SLAM2D::getMaximumLikelihoodTransform(int theta_index, rigid_t& rigid)
{
  // Define the steps of each parameter iteration
  double delta_x = rigid.res / 4;
  double delta_y = rigid.res / 4;
  double delta_theta = 0.5 * rigid.theta_factor * M_PI / 360;
  double sum;
  int index_x, index_y;
  Eigen::Matrix2Xd scan_aux, scan_aux2;
  double aux = theta_index * delta_theta + rigid.theta;
  Eigen::Rotation2Dd rot2(aux);

  scan_aux.resize(2, rigid.scan_in.cols());

  // Compute the rotation first
  if(aux != 0)
    scan_aux2 = rot2.toRotationMatrix() * rigid.scan_in;
  else
    scan_aux2 = rigid.scan_in;
    
  // Then iterate over the traslations
  for (int x_index = -x_iteration_; x_index <= x_iteration_; x_index++)
  {
    // Compute x traslaton
    scan_aux.row(0) = scan_aux2.row(0) + Eigen::MatrixXd::Constant(1, scan_aux2.cols(), rigid.x + delta_x * x_index);
    for (int y_index = -y_iteration_; y_index <= y_iteration_; y_index++)
    {
      // Compute y traslation
      scan_aux.row(1) = scan_aux2.row(1) + Eigen::MatrixXd::Constant(1, scan_aux2.cols(), rigid.y + delta_y * y_index);

      // Get values of points in map
      sum = 0.0;
      for (int m = 0; m < scan_aux.cols(); m++)
      {
        index_x = int(round((m_ / 2 + scan_aux(0, m)) / rigid.res));
        index_y = int(round((n_ / 2 + scan_aux(1, m)) / rigid.res));
        sum += pow((1 - rigid.map(index_x, index_y)), 2);
      }

      // If the sum is the smallest so far, save the parameters
      laser_processing_mutex_.lock();
      if (sum < rigid.sum_out)
      {
        rigid.x_out = rigid.x + x_index * delta_x;
        rigid.y_out = rigid.y + y_index * delta_y;
        rigid.theta_out = rigid.theta + theta_index * delta_theta;
        rigid.sum_out = sum;
        rigid.scan_out = scan_aux;
      }
      laser_processing_mutex_.unlock();
    }
  }

  return 0;
}

/**
 * @brief Gets points which belong to a contour from LaserScan data, taking into account max and min range values
 *
 * @param scan data from the laser scan.
 * @param points_out Matrix with extracted points
 * @return int 0: ok
 */
int SLAM2D::getPointsFromScan(sensor_msgs::LaserScan scan, Eigen::Matrix2Xd &points_out)
{
  Eigen::Matrix2Xd scan_points = Eigen::Matrix2Xd::Constant(2, scan.ranges.size(), 0.0);
  int points_count = 0, points_aux = 0, i;
  std::vector<double> angles;
  double pointmod_reg, pointmod_next, pointmod_res;
  bool first_time = true;
  
  points_out = Eigen::Matrix2Xd::Constant(2, scan.ranges.size(), 0.0);
  
  for (double angle = scan.angle_min; angle < scan.angle_max; angle += scan.angle_increment)
  {
    angles.push_back(angle);
  }

  // Get the points that belong to contours
  for (i = 0; i < scan.ranges.size(); i++)
  {
    // if (scan.ranges[i] < scan.range_max && scan.ranges[i] > scan.range_min && !isinf(scan.ranges[i]))
    if (scan.ranges[i] < 5.0 && scan.ranges[i] > scan.range_min && !isinf(scan.ranges[i]))
    {
      // Convert scan to point (x,y)
      scan_points(0, i) = -(sin(angles[i]) * scan.ranges[i]);
      scan_points(1, i) = cos(angles[i]) * scan.ranges[i];

      // The first time a valid scan point is captured, store the value only
      if(first_time)
        first_time = false;
      // Otherwise, continue operation
      else
      {
        // Evaluate if the point is part of a contour
        pointmod_reg = sqrt(pow(scan_points(0, i - 1), 2) + pow(scan_points(1, i - 1), 2));
        pointmod_next = sqrt(pow(scan_points(0, i), 2) + pow(scan_points(1, i), 2));

        if (pointmod_reg >= pointmod_next)
          pointmod_res = pointmod_reg - pointmod_next;
        else
          pointmod_res = pointmod_next - pointmod_reg;

        // If the rest of the module of the two contiguous points is less than the threshold, 
        //then they can be part of a contour
        if (pointmod_res < point_dis_threshold_)
        {
          // Save point i-1 in case its the starting point of a new contour
          if (points_aux == 0)
          {
            points_out.col(points_count + points_aux) = scan_points.col(i - 1);
            points_aux++;
          }
          points_out.col(points_count + points_aux) = scan_points.col(i);
          points_aux++;
        }
        else
        {
          // If there are at least min_adjacent_points_ contiguous points, 
          //they are considered to belong to a valid contour.
          if (points_aux >= min_adjacent_points_)
          {
            // Move points index points_aux times, otherwise they will be ovewritten
            points_count += points_aux;
          }
          // Restart points_aux
          points_aux = 0;
        }
      }
    }
  }
  // In case the last points in the scan where part of a contour
  if (points_aux >= min_adjacent_points_)
    points_count += points_aux;

  // Resize the output matrix
  points_out.conservativeResize(2, points_count);

  return 0;
}

/**
 * @brief Computes rigid body transformations of the lidar data and returns the one that best matches with the map
 * M(t-1)
 *
 * @param scan_in Input laser scanner
 * @param scan_out Laser scanner transformed
 */
void SLAM2D::getRigidBodyTransform(const Eigen::Ref<const Eigen::Matrix2Xd> scan_in, Eigen::Matrix2Xd &scan_out)
{
  rigid_t rigid;

  std::vector<boost::thread*> rigid_body_threads;

  rigid.scan_in = scan_in;
  rigid.x = x_;
  rigid.y = y_;

  if(use_imu_)
    rigid.theta = laser_imu_theta_;
  else
    rigid.theta = theta_;

  for(int map_index = 0; map_index < res_vec_.size(); map_index++)
  {
    rigid.sum_out = 10000.0;
    // Start with the low resolution map
    rigid.theta_factor = res_vec_(map_index) / res_vec_(0);
    rigid.res = res_vec_(map_index);
    rigid.map = map_eig_vec_[map_index];
    for (int theta_index = -theta_iteration_; theta_index <= theta_iteration_; theta_index++)
    {
      // Get the best rigid body transform, as the index being part of delta x
      rigid_body_threads.push_back(
          new boost::thread(&SLAM2D::getMaximumLikelihoodTransform, this, theta_index, boost::ref(rigid)));
    }

    // delete created threads
    for (int i = 0; i < rigid_body_threads.size(); i++)
    {
      rigid_body_threads[i]->join();
      delete rigid_body_threads[i];
    }

    rigid_body_threads.clear();

    // Get best transform for high resolution map, based on low resolution map
    rigid.x = rigid.x_out;
    rigid.y = rigid.y_out;
    rigid.theta = rigid.theta_out;
  }

  // Update pose data and output scan
  x_ = rigid.x_out;
  y_ = rigid.y_out;

  theta_ = rigid.theta_out;

  scan_out = rigid.scan_out;

  ROS_INFO("(x_, y_, theta_) = (%f, %f, %f)", x_, y_, theta_);
}

/**
 * @brief Callback function of the ground truth topic, to publish the real odometry as a path
 *
 * @param scanptr Pointer to ground truth data
 */
void SLAM2D::groundTruthCallback(const nav_msgs::Odometry::ConstPtr& ground_truth)
{
  static nav_msgs::Path real_path;
  nav_msgs::Odometry gt = *ground_truth;
  geometry_msgs::PoseStamped real_pose;
  real_pose.header.stamp = ros::Time::now();
  real_path.header.stamp = real_pose.header.stamp;
  real_pose.header.frame_id = "real_path";
  real_path.header.frame_id = "real_path";
  
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
 * @brief Callback function of the imu topic
 *
 * @param scanptr Pointer to imu data
 */
void SLAM2D::inertialCallback(const sensor_msgs::Imu::ConstPtr& imuptr)
{
  tf::Quaternion q(imuptr->orientation.x, imuptr->orientation.y, imuptr->orientation.z, imuptr->orientation.w);
  tf::Matrix3x3 m(q);
  double pitch, roll, yaw;
  m.getRPY(roll, pitch, yaw);
  imu_theta_ = yaw;
  got_imu_data_ = true;
}

/**
 * @brief Initializes the private variables of the class
 *
 */
void SLAM2D::init(double res)
{
  double std_dev;

  started_ = false;

  laser_count_ = 0;
  transform_publish_period_ = 0.05;

  point_free_ = 0.3;
  point_noinfo_ = 0.5;
  point_occupied_ = 0.7;
  point_dis_threshold_ = 2 * res; //0.1 antes
  min_adjacent_points_ = 5;

  x_iteration_ = 5;
  y_iteration_ = 5;
  theta_iteration_ = 5;

  // Define slam frames in case they were not given by user
  if (!nh_.getParam("base_frame", base_frame_))
    base_frame_ = "base_link";
  if (!nh_.getParam("map_frame", map_frame_))
    map_frame_ = "map";
  if (!nh_.getParam("imu_frame", imu_frame_))
    imu_frame_ = "imu";
  if (!nh_.getParam("tf_delay", tf_delay_))
    tf_delay_ = transform_publish_period_;
  if (!nh_.getParam("scan_topic_name", scan_topic_name_))
    scan_topic_name_ = "/scan";
  if (!nh_.getParam("imu_topic_name", imu_topic_name_))
    use_imu_ = false;
  else
    use_imu_ = true;
  if (!nh_.getParam("ground_truth_topic_name", ground_truth_topic_name_))
    publish_ground_truth_ = false;
  else
    publish_ground_truth_ = true;
  if (!nh_.getParam("map_count", map_count_))
    map_count_ = 3;
  if (!nh_.getParam("std_dev", std_dev))
    std_dev = 0.1;
  if (!nh_.getParam("publish_map_to_laser", publish_map_to_laser_))
    publish_map_to_laser_ = false;
  else
    publish_map_to_laser_ = true;
  if(!nh_.getParam("extodometry_topic_name", extodometry_topic_name_))
    use_ext_odometry_ = false;
  else
    use_ext_odometry_ = true;

  ROS_INFO("Params:");
  std::cout << "base_frame: " << base_frame_ << std::endl;
  std::cout << "map_frame: " << map_frame_ << std::endl;
  std::cout << "imu_frame: " << imu_frame_ << std::endl;
  std::cout << "tf_delay: " << tf_delay_ << std::endl;
  std::cout << "scan_topic_name: " << scan_topic_name_ << std::endl;
  if(use_imu_)
  { 
    std::cout << "use_imu: true" << std::endl;
    std::cout << "imu_topic_name: " << imu_topic_name_ << std::endl;
  }
  else
    std::cout << "use_imu: false" << std::endl;
  if(publish_ground_truth_)
  {
    std::cout << "publish_ground_truth: true" << std::endl;
    std::cout << "ground_truth_topic_name: " << ground_truth_topic_name_ << std::endl;
  }
  else
    std::cout << "publish_ground_truth: false" << std::endl;

  std::cout << "publish_map_to_laser: " << publish_map_to_laser_ << std::endl;
  std::cout << "map_count: " << map_count_ << std::endl;
  std::cout << "std_dev: " << std_dev << std::endl;

  // Set the resolution vector
  res_vec_.resize(map_count_,1);
  res_multipliers_.resize(map_count_,1);
  for(int n = (map_count_-1); n > 0; n--)
    res_multipliers_((map_count_ - 1) - n) = 2 * n;
  res_multipliers_(map_count_-1) = 1;
  res_vec_ << res_multipliers_ * res;

  for(int i = 0; i < res_vec_.size(); i++)
    map_eig_vec_.push_back(Eigen::MatrixXd::Constant(uint32_t(round(m_ / res_vec_(res_vec_.size()-1))), uint32_t(round(n_ / res_vec_(res_vec_.size()-1))), point_noinfo_));

  std_dev_(0) = std_dev;
  std_dev_(1) = std_dev;

  x_ = 0.0;
  y_ = 0.0;
  theta_ = 0.0;
  ext_x_ = 0.0;
  ext_y_ = 0.0;
  ext_theta_ = 0.0;
  imu_theta_ = 0.0;
  
  got_imu_data_ = false;
  got_ext_odom_ = false;

  occmap_.info.width = round(n_ / res_vec_(res_vec_.size()-1));
  occmap_.info.height = round(m_ / res_vec_(res_vec_.size()-1));
  occmap_.info.resolution = res_vec_(res_vec_.size()-1);
  occmap_.info.origin.position.x = -0.5 * m_;
  occmap_.info.origin.position.y = -0.5 * n_;
  occmap_.info.origin.position.z = 0;
  
  occmap_.header.frame_id = map_frame_;

  for (int i = 0; i < round((m_ * n_) / (res_vec_(res_vec_.size()-1) * res_vec_(res_vec_.size()-1))); i++)
    occmap_.data.push_back(-1);

  odom_.header.stamp = ros::Time::now();
  odom_.header.frame_id = "robot_odom2d";
  odom_.twist.twist.angular.x = 0.0;
  odom_.twist.twist.angular.y = 0.0;
  odom_.twist.twist.angular.z = 0.0;
  odom_.twist.twist.linear.x = 0.0;
  odom_.twist.twist.linear.y = 0.0;
  odom_.twist.twist.linear.z = 0.0;
  odom_.pose.pose.orientation.x = 0.0;
  odom_.pose.pose.orientation.y = 0.0;
  odom_.pose.pose.orientation.z = 0.0;
  odom_.pose.pose.orientation.w = 1.0;
  odom_.pose.pose.position.x = 0.0;
  odom_.pose.pose.position.y = 0.0;
  odom_.pose.pose.position.z = 0.0;
}

/**
 * @brief Callback function of the laser scan topic
 *
 * @param scanptr Pointer to laser scan data
 */
void SLAM2D::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scanptr)
{
  static int first_time = 1;
  static ros::Time last = ros::Time::now();
  static nav_msgs::Path path;
  static nav_msgs::Odometry odom;
  
  if(got_ext_odom_)
  {
    x_ = ext_x_;
    y_ = ext_y_;
    theta_ = ext_theta_;
  }
  
  ros::Time now = ros::Time::now();
  geometry_msgs::PoseStamped pose;
  sensor_msgs::LaserScan scan = *scanptr;

  Eigen::Matrix2Xd scan_points, scan_transformed;

  // Rotation matrix used only the first time the laser scan data is stored
  Eigen::Rotation2Dd rot2(imu_theta_);

  // Para calcular tiempos del algoritmo
  ros::WallTime start, end;

  pose.header.frame_id = "robot_path2d";
  path.header.frame_id = "robot_path2d";
  
  // odom_.header.frame_id = "robot_odom2d";
  odom_.header.frame_id = "odom2d";
  odom_.child_frame_id = "base_link";

  if (first_time)
  {
    first_time = 0;
    return;
  }

  // In case the IMU is used, first we need the data from that sensor
  if(use_imu_ && !got_imu_data_)
    return;

  laser_imu_theta_ = imu_theta_;
  got_imu_data_ = false;
  
  start = ros::WallTime::now();

  // Get all valid points
  this->getPointsFromScan(scan, scan_points);

  laser_count_++;

  // If laser count == 1, store data, otherwise get the rigid body transform
  if (laser_count_ > 1)
  {
    this->getRigidBodyTransform(scan_points, scan_transformed);
  }
  else
  {
    if(use_imu_)
    {
      // Rotate data
      scan_transformed = rot2.toRotationMatrix() * scan_points;
    }
    else
      scan_transformed = scan_points;
  }

  last = now;

  this->mapsUpdate(scan_transformed);

  end = ros::WallTime::now();
  double execution_time = (end - start).toNSec() * 1e-6;
  ROS_INFO_STREAM_COND(debug_ < 0, "Exectution time (ms): " << execution_time);
  map_pub_.publish(occmap_);


  path.header.stamp = now;
  pose.header.stamp = now;
  pose.pose.position.x = x_;
  pose.pose.position.y = y_;
  pose.pose.position.z = 0;
  pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta_);

  geometry_msgs::Quaternion qt_reg = tf::createQuaternionMsgFromYaw(theta_);
  geometry_msgs::Quaternion qt_prev = odom_.pose.pose.orientation;

  tf::Quaternion qt_tf_reg, qt_tf_prev;
  tf::quaternionMsgToTF(qt_reg, qt_tf_reg);
  tf::quaternionMsgToTF(qt_prev, qt_tf_prev);

  tfScalar angle_reg = qt_tf_reg.getAngle();
  tf::Vector3 axis_reg = qt_tf_reg.getAxis();
  tfScalar angle_prev = qt_tf_prev.getAngle();
  tf::Vector3 axis_prev = qt_tf_prev.getAxis();

  tf::Vector3 angaxis_reg = angle_reg * axis_reg;
  tf::Vector3 angaxis_prev = angle_prev * axis_prev;

  odom_.twist.twist.linear.x = (x_ - odom_.pose.pose.position.x) / (now - odom_.header.stamp).toSec();
  odom_.twist.twist.linear.y = (y_ - odom_.pose.pose.position.y) / (now - odom_.header.stamp).toSec();

  odom_.twist.twist.angular.x = (angaxis_reg.getX() - angaxis_prev.getX()) / (now - odom_.header.stamp).toSec();
  odom_.twist.twist.angular.y = (angaxis_reg.getY() - angaxis_prev.getY()) / (now - odom_.header.stamp).toSec();
  odom_.twist.twist.angular.z = (angaxis_reg.getZ() - angaxis_prev.getZ()) / (now - odom_.header.stamp).toSec();
  
  odom_.header.stamp = now;
  odom_.pose.pose.position.x = x_;
  odom_.pose.pose.position.y = y_;

  odom_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta_);

  for(int i = 0; i < 36; i++)
  {
    if(i != 0 && i != 7 && i !=14 && i != 21 && i != 28 && i != 35)
    {
      odom_.pose.covariance[i] = 0;
      odom_.twist.covariance[i] = 0;
    }
    else
    {
      odom_.pose.covariance[i] = 0.05;
      odom_.twist.covariance[i] = 0.05;
    }
  }

  path.poses.push_back(pose);
  path_pub_.publish(path);
  odom_pub_.publish(odom_);
}

/**
 * @brief Updates saved maps
 *
 * @param scan_points Scan points
 */
void SLAM2D::mapsUpdate(Eigen::Matrix2Xd scan_points)
{
  Eigen::MatrixXd m;
  // Index of each point in map
  Eigen::Vector2i index_point;
  // Points free between the vehicle and the measured obstacle
  std::vector<geometry_msgs::Point32> points_free;
  // Point to be updated by the logit function in each step
  Eigen::Vector2d point_update;
  // Max and min values of the scan point vector
  Eigen::Vector2d max_val_rows = scan_points.rowwise().maxCoeff();
  Eigen::Vector2d min_val_rows = scan_points.rowwise().minCoeff();

  // Map min size in x and y
  Eigen::Vector2d map_size;
  Eigen::Vector2i map_delta;
  
  double gauss;

  // Update each submap
  for(int map_index = 0; map_index < res_vec_.size(); map_index++)
  {
    /* FIXME: why is res_ not enough? */
    // Get the max value of rows and cols
    if (std::abs(max_val_rows(0)) >= std::abs(min_val_rows(0)))
      map_size(0) = std::abs(max_val_rows(0)) + 10 * res_vec_(map_index);
    else
      map_size(0) = std::abs(min_val_rows(0)) + 10 * res_vec_(map_index);

    if (std::abs(max_val_rows(1)) >= std::abs(min_val_rows(1)))
      map_size(1) = std::abs(max_val_rows(1)) + 10 * res_vec_(map_index);
    else
      map_size(1) = std::abs(min_val_rows(1)) + 10 * res_vec_(map_index);

    // Init map with no info value
    m = Eigen::MatrixXd::Constant(uint32_t(round(2 * map_size(0) / res_vec_(map_index))), uint32_t(round(2 * map_size(1) / res_vec_(map_index))),
                                  point_noinfo_);

    // Update current map
    for (int i = 0; i < scan_points.cols(); i++)
    {
      // Get the free points between the vehicle and each laser point
      int ret_bres = this->bresenhamLineAlgorithm(x_ / res_vec_(map_index), 
                                                  y_ / res_vec_(map_index), 
                                                  scan_points(0, i) / res_vec_(map_index), 
                                                  scan_points(1, i) / res_vec_(map_index), 
                                                  points_free);

      if (points_free.size() > 1)
      {
        // As we update around the point, obtaining a 3x3 matrix, points_free.size()-2 is needed instead of points_free.size()-1
        for (int index = 0; index < points_free.size(); index++)
        {
          // Update free points in map
          // Center of map + point
          index_point(0) = uint32_t(round(round(map_size(0) / res_vec_(map_index)) + points_free[index].x));
          index_point(1) = uint32_t(round(round(map_size(1) / res_vec_(map_index)) + points_free[index].y));

          if(index == points_free.size()-2)
            gauss = point_occupied_;
          else
            gauss = abs(this->gaussianBlur1D(points_free[index].x, points_free[points_free.size()-1].x, std_dev_(0)) *
                        this->gaussianBlur1D(points_free[index].y, points_free[points_free.size()-1].y, std_dev_(1)));

          m(index_point(0), index_point(1)) =
              this->getProbaFromLogit(this->getLogitFromProba(gauss) +
                                      this->getLogitFromProba(m(index_point(0), index_point(1))) - getLogitFromProba(point_noinfo_));
        }
      }
      index_point(0) = uint32_t(round(round(map_size(0) / res_vec_(map_index)) + scan_points(0, i) / res_vec_(map_index)));
      index_point(1) = uint32_t(round(round(map_size(1) / res_vec_(map_index)) + scan_points(1, i) / res_vec_(map_index)));

      // Clear free points vector
      points_free.clear();
    }

    // Get center of map
    map_delta(0) = int(round(0.5 * (m_ / res_vec_(map_index) - m.rows())));
    map_delta(1) = int(round(0.5 * (n_ / res_vec_(map_index) - m.cols())));

    for (int x = 0; x < m.rows(); x++)
    {
      for (int y = 0; y < m.cols(); y++)
      {
        // Update stored map with current measurement
        map_eig_vec_[map_index](x + map_delta(0), y + map_delta(1)) = this->getProbaFromLogit(
            this->getLogitFromProba(m(x, y)) +
            this->getLogitFromProba(map_eig_vec_[map_index](x + map_delta(0), y + map_delta(1))) - this->getLogitFromProba(point_noinfo_));

        // Finally, update the showed map in rviz (usually the high resolution map)
        if(map_index == res_vec_.size()-1)
        {
          occmap_.data[MAP_IDX(occmap_.info.width, x + map_delta(0), y + map_delta(1))] =
              int8_t(map_eig_vec_[map_index](x + map_delta(0), y + map_delta(1)) * 100);
        }
      }
    }
  }
}

/**
 * @brief Callback function of the external odometry of the vehicle
 *
 */
void SLAM2D::extOdometryCallback(const nav_msgs::Odometry::ConstPtr& odomptr)
{
  nav_msgs::Odometry ext_odom = *odomptr;

  got_ext_odom_ = true;
  ext_x_ = ext_odom.pose.pose.position.x;
  ext_y_ = ext_odom.pose.pose.position.y;
  ext_theta_ = tf::getYaw(ext_odom.pose.pose.orientation);

}

/**
 * @brief Publishes the transformed point of the vehicle
 *
 */
void SLAM2D::publishTransform(void)
{
  if (transform_publish_period_ == 0)
    return;
  ros::Time tf_expiration;
  ros::Rate r(1.0 / transform_publish_period_);
  while (ros::ok())
  {
    map_to_laser_ = tf::Transform(tf::createQuaternionFromYaw(theta_), tf::Vector3(x_, y_, 0.0));
    tf_expiration = ros::Time::now() + ros::Duration(tf_delay_);
    tf_broadcaster_.sendTransform(tf::StampedTransform(map_to_laser_, tf_expiration, map_frame_, base_frame_));
    
    r.sleep();
  }
}
