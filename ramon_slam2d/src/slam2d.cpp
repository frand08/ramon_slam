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
  float res = 0.02;

  res_vec_.resize(11,1);
  res_multipliers_.resize(11,1);
  res_multipliers_ << 20,
                      18,
                      16,
                      14,
                      12,
                      10,
                      8,
                      6,
                      4,
                      2,
                      1;

  res_vec_ << res_multipliers_ * res;

  res_high_ = 0.02;
  res_med_ = 0.1;
  res_low_ = 0.2;
  this->init();
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
  res_high_ = res;
  res_med_ = 5 * res;
  res_low_ = 10 * res;

  res_vec_.resize(11,1);
  res_vec_ << 20 * res, 
              18 * res, 
              16 * res, 
              14 * res, 
              12 * res, 
              10 * res, 
              8 * res, 
              6 * res, 
              4 * res, 
              2 * res, 
              res;

  this->init();
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
  map_meta_pub_ = nh_.advertise<nav_msgs::MapMetaData>("map_metadata", 1);
  laser_sub_ = nh_.subscribe(scan_topic_name_, 1, &SLAM2D::laserCallback, this);  // check the last param
  if(use_imu_)
    imu_sub_ = nh_.subscribe(imu_topic_name_, 1, &SLAM2D::inertialCallback, this);
  // TF thread
  transform_thread_ = new boost::thread(boost::bind(&SLAM2D::publishTransform, this));
}

/* Private functions */

/**
 * @brief Gets the maximum likelihood transform for a given delta x
 *
 * @param x_index delta x
 * @param rigid Information about the input scan, offsets and output scans (reference)
 * @return int zero value
 */
int SLAM2D::getMaximumLikelihoodTransform(int x_index, rigid_t& rigid)
{
  double delta_x;
  double delta_y;
  double delta_theta;
  double sum;
  int index_x, index_y;
  Eigen::Matrix2Xd scan_aux;
  scan_aux.resize(2, scan_aux.cols());
  delta_x = rigid.res / 2;
  delta_y = rigid.res / 2;
  delta_theta = rigid.theta_factor * M_PI / 360;

  for (int y_index = -10; y_index <= 10; y_index++)
  {
    // if(!use_imu_)
    {
      for (int theta_index = -5; theta_index <= 5; theta_index++)
      {
        scan_aux = this->rotateAndTranslate2D(rigid.scan_in, (rigid.x + x_index * delta_x), (rigid.y + y_index * delta_y),
                                              (rigid.theta + theta_index * delta_theta));
        sum = 0.0;
        for (int m = 0; m < scan_aux.cols(); m++)
        {
          index_x = int(round((m_ / 2 + scan_aux(0, m)) / rigid.res));
          index_y = int(round((n_ / 2 + scan_aux(1, m)) / rigid.res));
          sum += pow((1 - rigid.map(index_x, index_y)), 2);
        }

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
    // else
    // {
    //   scan_aux = this->rotateAndTranslate2D(rigid.scan_in, (rigid.x + x_index * delta_x), (rigid.y + y_index * delta_y), 0.0);
    //   sum = 0.0;
    //   for (int m = 0; m < scan_aux.cols(); m++)
    //   {
    //     index_x = int(round((m_ / 2 + scan_aux(0, m)) / rigid.res));
    //     index_y = int(round((n_ / 2 + scan_aux(1, m)) / rigid.res));
    //     sum += pow((1 - rigid.map(index_x, index_y)), 2);
    //   }

    //   laser_processing_mutex_.lock();
    //   if (sum < rigid.sum_out)
    //   {
    //     rigid.x_out = rigid.x + x_index * delta_x;
    //     rigid.y_out = rigid.y + y_index * delta_y;
    //     rigid.sum_out = sum;
    //     rigid.scan_out = scan_aux;
    //   }
    //   laser_processing_mutex_.unlock();
    // }
    
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
    if (scan.ranges[i] < scan.range_max && scan.ranges[i] > scan.range_min && !isinf(scan.ranges[i]))
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

        // If the rest of the module of the two contiguous points is less than the threshold, then they can be part of a
        // contour
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
          // If there are at least min_adjacent_points_ contiguous points, they are considered to belong to a valid
          // contour.
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
  double x_out, y_out, theta_out;
  double sum_out;
  int i;
  rigid_t rigid;
  Eigen::Rotation2Dd rot2(laser_imu_theta_);

  std::vector<boost::thread*> rigid_body_threads_high, rigid_body_threads_med, rigid_body_threads_low;
  // if(use_imu_)
    // rigid.scan_in = rot2.toRotationMatrix() * scan_in;
  // else
    rigid.scan_in = scan_in;
  rigid.x = x_;
  rigid.y = y_;
  if(use_imu_)
    rigid.theta = laser_imu_theta_;
  else
    rigid.theta = theta_;
  rigid.theta_factor = 1.0;
  rigid.sum_out = 10000.0;

  // Start with the low resolution map
  rigid.res = res_low_;
  rigid.map = map_eig_low_;
  for (i = -10; i <= 10; i++)
  {
    // Get the best rigid body transform, as the index being part of delta x
    rigid_body_threads_low.push_back(
        new boost::thread(&SLAM2D::getMaximumLikelihoodTransform, this, i, boost::ref(rigid)));
  }

  // delete created threads
  for (i = 0; i < rigid_body_threads_low.size(); i++)
  {
    rigid_body_threads_low[i]->join();
    delete rigid_body_threads_low[i];
  }

  // Get best transform for high resolution map, based on low resolution map
  rigid.x = rigid.x_out;
  rigid.y = rigid.y_out;
  rigid.theta = rigid.theta_out;
  ROS_INFO("(x, y, theta) LOW = (%f, %f, %f)", rigid.x, rigid.y, rigid.theta);
  rigid.sum_out = 10000.0;
  rigid.res = res_med_;
  rigid.theta_factor = 0.5;
  rigid.map = map_eig_med_;
  for (i = -10; i <= 10; i++)
  {
    // Get the best rigid body transform, as the index being part of delta x
    rigid_body_threads_med.push_back(
        new boost::thread(&SLAM2D::getMaximumLikelihoodTransform, this, i, boost::ref(rigid)));
  }

  // delete created threads
  for (i = 0; i < rigid_body_threads_med.size(); i++)
  {
    rigid_body_threads_med[i]->join();
    delete rigid_body_threads_med[i];
  }

  // Get best transform for high resolution map, based on low resolution map
  rigid.x = rigid.x_out;
  rigid.y = rigid.y_out;
  rigid.theta = rigid.theta_out;
  ROS_INFO("(x, y, theta) MED = (%f, %f, %f)", rigid.x, rigid.y, rigid.theta);
  rigid.sum_out = 10000.0;
  rigid.res = res_high_;
  rigid.theta_factor = 0.1;
  rigid.map = map_eig_high_;
  for (i = -10; i <= 10; i++)
  {
    // Get the best rigid body transform, as the index being part of delta x
    rigid_body_threads_high.push_back(
        new boost::thread(&SLAM2D::getMaximumLikelihoodTransform, this, i, boost::ref(rigid)));
  }

  // delete created threads
  for (i = 0; i < rigid_body_threads_high.size(); i++)
  {
    rigid_body_threads_high[i]->join();
    delete rigid_body_threads_high[i];
  }

  // Update pose data and output scan
  x_ = rigid.x_out;
  y_ = rigid.y_out;

  // if(use_imu_)
  //   theta_ = laser_imu_theta_;
  // else
    theta_ = rigid.theta_out;

  scan_out = rigid.scan_out;

  ROS_INFO("(x_, y_, theta_) = (%f, %f, %f)", x_, y_, theta_);
}

/**
 * @brief Computes rigid body transformations of the lidar data and returns the one that best matches with the map
 * M(t-1)
 *
 * @param scan_in Input laser scanner
 * @param scan_out Laser scanner transformed
 */
void SLAM2D::getRigidBodyTransform(const Eigen::Ref<const Eigen::Matrix2Xd> scan_in, Eigen::Matrix2Xd &scan_out, int mongo)
{
  double x_out, y_out, theta_out;
  double sum_out;
  int i;
  rigid_t rigid;
  Eigen::Rotation2Dd rot2(laser_imu_theta_);

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
    for (i = -10; i <= 10; i++)
    {
      // Get the best rigid body transform, as the index being part of delta x
      rigid_body_threads.push_back(
          new boost::thread(&SLAM2D::getMaximumLikelihoodTransform, this, i, boost::ref(rigid)));
    }

    // delete created threads
    for (i = 0; i < rigid_body_threads.size(); i++)
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
 * @brief Initializes the private variables of the class
 *
 */
void SLAM2D::init()
{
  laser_count_ = 0;
  transform_publish_period_ = 0.05;

  point_free_ = 0.3;
  point_noinfo_ = 0.5;
  point_occupied_ = 0.7;
  point_dis_threshold_ = 0.1;
  min_adjacent_points_ = 5;

  map_eig_high_ = Eigen::MatrixXd::Constant(uint32_t(round(m_ / res_high_)), uint32_t(round(n_ / res_high_)), point_noinfo_);
  map_eig_med_ = Eigen::MatrixXd::Constant(uint32_t(round(m_ / res_med_)), uint32_t(round(n_ / res_med_)), point_noinfo_);
  map_eig_low_ = Eigen::MatrixXd::Constant(uint32_t(round(m_ / res_low_)), uint32_t(round(n_ / res_low_)), point_noinfo_);

  for(int i = 0; i < res_vec_.size(); i++)
    map_eig_vec_.push_back(Eigen::MatrixXd::Constant(uint32_t(round(m_ / res_vec_(res_vec_.size()-1))), uint32_t(round(n_ / res_vec_(res_vec_.size()-1))), point_noinfo_));

  std_dev_(0) = 0.01;
  std_dev_(1) = 0.01;

  x_ = 0.0;
  y_ = 0.0;
  theta_ = 0.0;
  imu_theta_ = 0.0;
  
  got_imu_data_ = false;

  // Init occupancygrid map
  // occmap_.info.width = n_ / res_high_;
  // occmap_.info.height = m_ / res_high_;
  // occmap_.info.resolution = res_high_;
  // occmap_.info.origin.position.x = -0.5 * m_;
  // occmap_.info.origin.position.y = -0.5 * n_;
  // occmap_.info.origin.position.z = 0;
  
  // for (int i = 0; i < (m_ * n_) / (res_high_ * res_high_); i++)
  //   occmap_.data.push_back(-1);

  occmap_.info.width = n_ / res_vec_(res_vec_.size()-1);
  occmap_.info.height = m_ / res_vec_(res_vec_.size()-1);
  occmap_.info.resolution = res_vec_(res_vec_.size()-1);
  occmap_.info.origin.position.x = -0.5 * m_;
  occmap_.info.origin.position.y = -0.5 * n_;
  occmap_.info.origin.position.z = 0;
  
  for (int i = 0; i < (m_ * n_) / (res_vec_(res_vec_.size()-1) * res_vec_(res_vec_.size()-1)); i++)
    occmap_.data.push_back(-1);

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

  ROS_INFO_STREAM("use imu:" << use_imu_);
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
  ros::Time now = ros::Time::now();
  sensor_msgs::LaserScan scan = *scanptr;

  Eigen::Matrix2Xd scan_points, scan_transformed;

  // Para calcular tiempos del algoritmo
  ros::WallTime start, end;

  Eigen::Rotation2Dd rot2(imu_theta_);

  laser_imu_theta_ = imu_theta_;

  if (first_time)
  {
    first_time = 0;
    return;
  }

  // In case the IMU is used, first we need the data from that sensor
  if(use_imu_ && !got_imu_data_)
    return;

  got_imu_data_ = false;
  
  start = ros::WallTime::now();

  // Get all valid points
  this->getPointsFromScan(scan, scan_points);

  laser_count_++;

  // If laser count == 1, store data, otherwise get the rigid body transform
  if (laser_count_ > 1)
  {
    // if(now.sec - last.sec < 1)
    // {
    //   return;      
    // }
    this->getRigidBodyTransform(scan_points, scan_transformed,1);
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

  this->mapUpdate(scan_transformed,1);

  end = ros::WallTime::now();
  double execution_time = (end - start).toNSec() * 1e-6;
  ROS_INFO_STREAM_COND(debug_ < 0, "Exectution time (ms): " << execution_time);
  map_pub_.publish(occmap_);
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
 * @brief Updates saved map
 *
 * @param scan_points Scan points
 */
void SLAM2D::mapUpdate(Eigen::Matrix2Xd scan_points)
{
  Eigen::MatrixXd m_high, m_med, m_low;
  // Index of each point in map
  Eigen::Vector2i index_high, index_med, index_low;
  // Points free between the vehicle and the measured obstacle
  std::vector<geometry_msgs::Point32> points_free_high, points_free_med, points_free_low;
  // Point to be updated by the logit function in each step
  Eigen::Vector2d point_update;
  // Max and min values of the scan point vector
  Eigen::Vector2d max_val_rows = scan_points.rowwise().maxCoeff();
  Eigen::Vector2d min_val_rows = scan_points.rowwise().minCoeff();

  // Map min size in x and y
  Eigen::Vector2d map_size_high, map_size_med, map_size_low;

  Eigen::Vector2i map_delta_high, map_delta_med, map_delta_low;

  /* FIXME: why is res_ not enough? */
  // Get the max value of rows and cols
  if (std::abs(max_val_rows(0)) >= std::abs(min_val_rows(0)))
  {
    map_size_high(0) = std::abs(max_val_rows(0)) + 10 * res_high_;
    map_size_med(0) = std::abs(max_val_rows(0)) + 10 * res_med_;
    map_size_low(0) = std::abs(max_val_rows(0)) + 10 * res_low_;
  }
  else
  {
    map_size_high(0) = std::abs(min_val_rows(0)) + 10 * res_high_;
    map_size_med(0) = std::abs(min_val_rows(0)) + 10 * res_med_;
    map_size_low(0) = std::abs(min_val_rows(0)) + 10 * res_low_;
  }

  if (std::abs(max_val_rows(1)) >= std::abs(min_val_rows(1)))
  {
    map_size_high(1) = std::abs(max_val_rows(1)) + 10 * res_high_;
    map_size_med(1) = std::abs(max_val_rows(1)) + 10 * res_med_;
    map_size_low(1) = std::abs(max_val_rows(1)) + 10 * res_low_;
  }
  else
  {
    map_size_high(1) = std::abs(min_val_rows(1)) + 10 * res_high_;
    map_size_med(1) = std::abs(min_val_rows(1)) + 10 * res_med_;
    map_size_low(1) = std::abs(min_val_rows(1)) + 10 * res_low_;
  }

  // Init map with no info value
  m_high = Eigen::MatrixXd::Constant(uint32_t(round(2 * map_size_high(0) / res_high_)), uint32_t(round(2 * map_size_high(1) / res_high_)),
                                point_noinfo_);
  m_med = Eigen::MatrixXd::Constant(uint32_t(round(2 * map_size_med(0) / res_med_)), uint32_t(round(2 * map_size_med(1) / res_med_)),
                                point_noinfo_);
  m_low = Eigen::MatrixXd::Constant(uint32_t(round(2 * map_size_low(0) / res_low_)), uint32_t(round(2 * map_size_low(1) / res_low_)),
                                point_noinfo_);

  // Update high resolution map
  for (int i = 0; i < scan_points.cols(); i++)
  {
    // Get the free points between the vehicle and each laser point
    this->bresenhamLineAlgorithm(0, 0, scan_points(0, i) / res_high_, scan_points(1, i) / res_high_, points_free_high);

    if (points_free_high.size() > 0)
    {
      for (int index = 0; index < points_free_high.size() - 1; index++)
      {
        // Update free points in map
        index_high(0) = uint32_t(round(round(map_size_high(0) / res_high_) + points_free_high[index].x));
        index_high(1) = uint32_t(round(round(map_size_high(1) / res_high_) + points_free_high[index].y));

        m_high(index_high(0), index_high(1)) =
            this->getProbaFromLogit(this->getLogitFromProba(point_free_) +
                                    this->getLogitFromProba(m_high(index_high(0), index_high(1))) - getLogitFromProba(point_noinfo_));
      }
    }
    index_high(0) = uint32_t(round(round(map_size_high(0) / res_high_) + scan_points(0, i) / res_high_));
    index_high(1) = uint32_t(round(round(map_size_high(1) / res_high_) + scan_points(1, i) / res_high_));

    // Update occupied points in map
    
    // Update around point option (3x3 matrix)
    // point_update(0) = scan_points(0, i) / res_high_;
    // point_update(1) = scan_points(1, i) / res_high_;
    // this->logitUpdate(m_high, index_high, point_update, std_dev_, res_high_);

    // One point update option
    m_high(index_high(0), index_high(1)) =
        this->getProbaFromLogit(this->getLogitFromProba(point_occupied_) +
                                this->getLogitFromProba(m_high(index_high(0), index_high(1))) - getLogitFromProba(point_noinfo_));

    // Clear free points vector
    points_free_high.clear();



    // Update med resolution map
    this->bresenhamLineAlgorithm(0, 0, scan_points(0, i) / res_med_, scan_points(1, i) / res_med_, points_free_med);

    if (points_free_med.size() > 0)
    {
      for (int index = 0; index < points_free_med.size() - 1; index++)
      {
        // Update free points in map
        index_med(0) = uint32_t(round(round(map_size_med(0) / res_med_) + points_free_med[index].x));
        index_med(1) = uint32_t(round(round(map_size_med(1) / res_med_) + points_free_med[index].y));

        m_med(index_med(0), index_med(1)) =
            this->getProbaFromLogit(this->getLogitFromProba(point_free_) +
                                    this->getLogitFromProba(m_med(index_med(0), index_med(1))) - getLogitFromProba(point_noinfo_));
      }
    }
    index_med(0) = uint32_t(round(round(map_size_med(0) / res_med_) + scan_points(0, i) / res_med_));
    index_med(1) = uint32_t(round(round(map_size_med(1) / res_med_) + scan_points(1, i) / res_med_));
    
    m_med(index_med(0), index_med(1)) =
        this->getProbaFromLogit(this->getLogitFromProba(point_occupied_) +
                                this->getLogitFromProba(m_med(index_med(0), index_med(1))) - getLogitFromProba(point_noinfo_));

    points_free_med.clear();



    // Update low resolution map
    this->bresenhamLineAlgorithm(0, 0, scan_points(0, i) / res_low_, scan_points(1, i) / res_low_, points_free_low);

    if (points_free_low.size() > 0)
    {
      for (int index = 0; index < points_free_low.size() - 1; index++)
      {
        // Update free points in map
        index_low(0) = uint32_t(round(round(map_size_low(0) / res_low_) + points_free_low[index].x));
        index_low(1) = uint32_t(round(round(map_size_low(1) / res_low_) + points_free_low[index].y));

        m_low(index_low(0), index_low(1)) =
            this->getProbaFromLogit(this->getLogitFromProba(point_free_) +
                                    this->getLogitFromProba(m_low(index_low(0), index_low(1))) - getLogitFromProba(point_noinfo_));
      }
    }
    index_low(0) = uint32_t(round(round(map_size_low(0) / res_low_) + scan_points(0, i) / res_low_));
    index_low(1) = uint32_t(round(round(map_size_low(1) / res_low_) + scan_points(1, i) / res_low_));
    
    m_low(index_low(0), index_low(1)) =
        this->getProbaFromLogit(this->getLogitFromProba(point_occupied_) +
                                this->getLogitFromProba(m_low(index_low(0), index_low(1))) - getLogitFromProba(point_noinfo_));

    points_free_low.clear();
  }


  map_delta_high(0) = int(round(0.5 * (m_ / res_high_ - m_high.rows())));
  map_delta_high(1) = int(round(0.5 * (n_ / res_high_ - m_high.cols())));

  map_delta_med(0) = int(round(0.5 * (m_ / res_med_ - m_med.rows())));
  map_delta_med(1) = int(round(0.5 * (n_ / res_med_ - m_med.cols())));

  map_delta_low(0) = int(round(0.5 * (m_ / res_low_ - m_low.rows())));
  map_delta_low(1) = int(round(0.5 * (n_ / res_low_ - m_low.cols())));

  for (int x = 0; x < m_high.rows(); x++)
  {
    for (int y = 0; y < m_high.cols(); y++)
    {
      // Update stored map with current measurement
      map_eig_high_(x + map_delta_high(0), y + map_delta_high(1)) = this->getProbaFromLogit(
          this->getLogitFromProba(m_high(x, y)) +
          this->getLogitFromProba(map_eig_high_(x + map_delta_high(0), y + map_delta_high(1))) - this->getLogitFromProba(point_noinfo_));


      if((m_med.rows() > x) && (m_med.cols() > y))
      {
        map_eig_med_(x + map_delta_med(0), y + map_delta_med(1)) = this->getProbaFromLogit(
            this->getLogitFromProba(m_med(x, y)) +
            this->getLogitFromProba(map_eig_med_(x + map_delta_med(0), y + map_delta_med(1))) - this->getLogitFromProba(point_noinfo_));
      }


      if((m_low.rows() > x) && (m_low.cols() > y))
      {
        map_eig_low_(x + map_delta_low(0), y + map_delta_low(1)) = this->getProbaFromLogit(
            this->getLogitFromProba(m_low(x, y)) +
            this->getLogitFromProba(map_eig_low_(x + map_delta_low(0), y + map_delta_low(1))) - this->getLogitFromProba(point_noinfo_));
      }

      occmap_.data[MAP_IDX(occmap_.info.width, x + map_delta_high(0), y + map_delta_high(1))] =
          int8_t(map_eig_high_(x + map_delta_high(0), y + map_delta_high(1)) * 100);
    }
  }
}


/**
 * @brief Updates saved map
 *
 * @param scan_points Scan points
 */
void SLAM2D::mapUpdate(Eigen::Matrix2Xd scan_points, int mongo)
{
  // Eigen::MatrixXd m_high, m_med, m_low;
  Eigen::MatrixXd m;
  // Index of each point in map
  // Eigen::Vector2i index_high, index_med, index_low;
  Eigen::Vector2i index_point;
  // Points free between the vehicle and the measured obstacle
  // std::vector<geometry_msgs::Point32> points_free_high, points_free_med, points_free_low;
  std::vector<geometry_msgs::Point32> points_free;
  // Point to be updated by the logit function in each step
  Eigen::Vector2d point_update;
  // Max and min values of the scan point vector
  Eigen::Vector2d max_val_rows = scan_points.rowwise().maxCoeff();
  Eigen::Vector2d min_val_rows = scan_points.rowwise().minCoeff();

  // Map min size in x and y
  // Eigen::Vector2d map_size_high, map_size_med, map_size_low;
  Eigen::Vector2d map_size;
  // Eigen::Vector2i map_delta_high, map_delta_med, map_delta_low;
  Eigen::Vector2i map_delta;
  
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

    // Update high resolution map
    for (int i = 0; i < scan_points.cols(); i++)
    {
      // Get the free points between the vehicle and each laser point
      this->bresenhamLineAlgorithm(0, 0, scan_points(0, i) / res_vec_(map_index), scan_points(1, i) / res_vec_(map_index), points_free);

      if (points_free.size() > 0)
      {
        for (int index = 0; index < points_free.size() - 1; index++)
        {
          // Update free points in map
          index_point(0) = uint32_t(round(round(map_size(0) / res_vec_(map_index)) + points_free[index].x));
          index_point(1) = uint32_t(round(round(map_size(1) / res_vec_(map_index)) + points_free[index].y));

          m(index_point(0), index_point(1)) =
              this->getProbaFromLogit(this->getLogitFromProba(point_free_) +
                                      this->getLogitFromProba(m(index_point(0), index_point(1))) - getLogitFromProba(point_noinfo_));
        }
      }
      index_point(0) = uint32_t(round(round(map_size(0) / res_vec_(map_index)) + scan_points(0, i) / res_vec_(map_index)));
      index_point(1) = uint32_t(round(round(map_size(1) / res_vec_(map_index)) + scan_points(1, i) / res_vec_(map_index)));

      // Update occupied points in map
      
      // Update around point option (3x3 matrix)
      // point_update(0) = scan_points(0, i) / res_vec_(map_index);
      // point_update(1) = scan_points(1, i) / res_vec_(map_index);
      // this->logitUpdate(m, index_point, point_update, std_dev_, res_vec_(map_index));

      // One point update option
      m(index_point(0), index_point(1)) =
          this->getProbaFromLogit(this->getLogitFromProba(point_occupied_) +
                                  this->getLogitFromProba(m(index_point(0), index_point(1))) - getLogitFromProba(point_noinfo_));

      // Clear free points vector
      points_free.clear();
    }

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
    map_to_laser_ = tf::Transform(tf::createQuaternionFromRPY(0, 0, theta_), tf::Vector3(x_, y_, 0.0));
    tf_expiration = ros::Time::now() + ros::Duration(tf_delay_);
    tf_broadcaster_.sendTransform(tf::StampedTransform(map_to_laser_, tf_expiration, map_frame_, base_frame_));
    r.sleep();
  }
}