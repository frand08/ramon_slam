/**
 * @file ramon_slam2d.cpp
 * @author Francisco A. Dominguez (dominguezfranciscoa@gmail.com)
 * @brief
 * @version 0.1
 * @date 2020-01-09
 *
 * @copyright Copyright (c) 2020
 *
 */

#include "ramon_slam2d.h"

/* Public Functions */

/**
 * @brief Construct a new RamonSlam2D::RamonSlam2D object
 *
 */
RamonSlam2D::RamonSlam2D() : transform_thread_(NULL), debug_(DEBUG_MODE)
{
  m_ = 50;
  n_ = 50;
  res_ = 0.02;
  this->init();
}

/**
 * @brief Construct a new RamonSlam2D::RamonSlam2D object
 *
 * @param M
 * @param N
 * @param res
 */
RamonSlam2D::RamonSlam2D(uint32_t M, uint32_t N, float res) : transform_thread_(NULL), debug_(DEBUG_MODE)
{
  m_ = M;
  n_ = N;
  res_ = res;
  this->init();
}

/**
 * @brief Destroy the RamonSlam2D::RamonSlam2D object
 *
 */
RamonSlam2D::~RamonSlam2D()
{
  if (transform_thread_)
  {
    transform_thread_->join();
    delete transform_thread_;
  }
}

/**
 * @brief
 *
 * @param x0
 * @param y0
 * @param x1
 * @param y1
 * @param point
 */
void RamonSlam2D::bresenhamLineAlgorithm(float x0, float y0, float x1, float y1,
                                         std::vector<geometry_msgs::Point32>& point)
{
  if (abs(y1 - y0) < abs(x1 - x0))
  {
    if (x0 > x1)
    {
      this->bresenhamLineLow(x1, y1, x0, y0, point);
    }
    else
    {
      this->bresenhamLineLow(x0, y0, x1, y1, point);
    }
  }
  else
  {
    if (y0 > y1)
    {
      this->bresenhamLineHigh(x1, y1, x0, y0, point);
    }
    else
    {
      this->bresenhamLineHigh(x0, y0, x1, y1, point);
    }
  }
}

/**
 * @brief Obtains the Gaussian Blur Integral
 *
 * @param a
 * @param b
 * @param c
 * @param std
 * @return float
 */
float RamonSlam2D::gaussianBlurIntegral(float a, float b, float c, float std)
{
  // Qian2019 - P.7
  // erf: error function
  return (-0.5 * (erf((c - b) / (sqrt(2) * std)) - erf((c - a) / (sqrt(2) * std))));
}

/**
 * @brief Gets the logit of the probability p, ie, logit(p) = log(p / (1 - p))
 *
 * @param value
 * @return float
 */
float RamonSlam2D::getLogitFromProba(float value)
{
  return log(value / (1 - value));
}

/**
 * @brief Gets the probability p from the logarithm of the odds p / (1 - p)
 *
 * @param value
 * @return float
 */
float RamonSlam2D::getProbaFromLogit(float value)
{
  return (std::exp(value) / (1 + std::exp(value)));
}

/**
 * @brief Rotates and translates a 2xN matrix
 *
 * @param matrix Matrix to be translated and rotated
 * @param x translation of x
 * @param y translation of y
 * @param theta rotation in radians
 * @return Eigen::Matrix2Xf
 */
Eigen::Matrix2Xf RamonSlam2D::rotateAndTranslate2D(Eigen::Matrix2Xf matrix, float x, float y, float theta)
{
  Eigen::Rotation2Df rot2(theta);
  Eigen::Matrix2Xf matrix_out;

  // First, do rotation
  matrix_out = rot2.toRotationMatrix() * matrix;

  // Then translation
  matrix_out.row(0) = matrix_out.row(0) + Eigen::MatrixXf::Constant(1, matrix_out.cols(), x);
  matrix_out.row(1) = matrix_out.row(1) + Eigen::MatrixXf::Constant(1, matrix_out.cols(), y);
  return matrix;
}

/**
 * @brief Begins the 2DSlam algorithm, advertising and subscribing to the needed topics
 *
 */
void RamonSlam2D::start()
{
  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1);
  map_meta_pub_ = nh_.advertise<nav_msgs::MapMetaData>("map_metadata", 1);
  laser_sub_ = nh_.subscribe("scan", 1, &RamonSlam2D::laserCallback, this);  // check the last param

  // TF thread
  transform_thread_ = new boost::thread(boost::bind(&RamonSlam2D::publishTransform, this));
}

/* Private functions */

/**
 * @brief
 *
 * @param x0
 * @param y0
 * @param x1
 * @param y1
 * @param point
 */
void RamonSlam2D::bresenhamLineHigh(float x0, float y0, float x1, float y1, std::vector<geometry_msgs::Point32>& point)
{
  float dx = x1 - x0;
  float dy = y1 - y0;
  float xi = 1;
  float D;
  float y, x;
  geometry_msgs::Point32 point_aux;

  if (dx < 0)
  {
    xi = -1;
    dx = -dx;
  }
  D = 2 * dx - dy;
  x = x0;

  for (y = y0; y < y1; y++)
  {
    // plot(x,y);
    point_aux.x = x;
    point_aux.y = y;
    point.push_back(point_aux);
    if (D > 0)
    {
      x = x + xi;
      D = D - 2 * dy;
    }
    D = D + 2 * dx;
  }
}

/**
 * @brief
 *
 * @param x0
 * @param y0
 * @param x1
 * @param y1
 * @param point
 */
void RamonSlam2D::bresenhamLineLow(float x0, float y0, float x1, float y1, std::vector<geometry_msgs::Point32>& point)
{
  float dx = x1 - x0;
  float dy = y1 - y0;
  float yi = 1;
  float D;
  float y, x;
  geometry_msgs::Point32 point_aux;

  if (dy < 0)
  {
    yi = -1;
    dy = -dy;
  }
  D = 2 * dy - dx;
  y = y0;

  point_aux.z = 0;
  for (x = x0; x < x1; x++)
  {
    point_aux.x = x;
    point_aux.y = y;
    point.push_back(point_aux);
    if (D > 0)
    {
      y = y + yi;
      D = D - 2 * dx;
    }
    D = D + 2 * dy;
  }
}

/**
 * @brief Gets the occupancy likelihood around a point (3x3 matrix). Gaussian assumption
 *
 * @param likelihood 3x3 matrix return data
 * @param point Point location
 */
void RamonSlam2D::getOccupancyLikelihood(Eigen::Matrix3f& likelihood, Eigen::Vector2f point)
{
  float Px1x2, Px2x3, Px3x4;  // x likelihood values
  float Py1y2, Py2y3, Py3y4;  // y likelihood values

  float x1, x2, x3, x4;  // x positions of square lines
  float y1, y2, y3, y4;  // y positions of square lines

  // x2 = (int(point.x / res_) * res_);
  x2 = point(0);
  x1 = x2 - res_;
  x3 = x2 + res_;
  x4 = x3 + res_;

  // y2 = (int(point.y / res_) * res_);
  y2 = point(1);
  y1 = y2 - res_;
  y3 = y2 + res_;
  y4 = y3 + res_;

  // Grid Point Occupation - 9 grid cells around the point using Gaussian assumption
  // x-coordinate occupancy likelihoods (Gaussian assumption)
  Px1x2 = this->gaussianBlurIntegral(x1, x2, point(0), std_x_);
  Px2x3 = this->gaussianBlurIntegral(x2, x3, point(0), std_x_);
  Px3x4 = this->gaussianBlurIntegral(x3, x4, point(0), std_x_);

  // y-coordinate occupancy likelihoods (Gaussian assumption)
  Py1y2 = this->gaussianBlurIntegral(y1, y2, point(1), std_y_);
  Py2y3 = this->gaussianBlurIntegral(y2, y3, point(1), std_y_);
  Py3y4 = this->gaussianBlurIntegral(y3, y4, point(1), std_y_);

  ROS_INFO_COND(debug_ > 0, "point=(%f;%f)", point(0), point(1));
  ROS_INFO_COND(debug_ > 0, "\nPx1x2 = %f\nPx2x3 = %f\nPx3x4 = %f", Px1x2, Px2x3, Px3x4);
  ROS_INFO_COND(debug_ > 0, "\nPy1y2 = %f\nPy2y3 = %f\nPy3y4 = %f", Py1y2, Py2y3, Py3y4);

  /* TODO: Px and Py gave wrong values? */
  /*
  // Compute the likelihood of the 9 grid cells around the laser point
  likelihood(0, 0) = point_occupied_ * (1 - abs(Px1x2 * Py3y4));  // 0
  likelihood(0, 1) = point_occupied_ * (1 - abs(Px2x3 * Py3y4));  // 1
  likelihood(0, 2) = point_occupied_ * (1 - abs(Px3x4 * Py3y4));  // 2

  likelihood(1, 0) = point_occupied_ * (1 - abs(Px1x2 * Py2y3));  // 3
  likelihood(1, 1) = point_occupied_ * (1 - abs(Px2x3 * Py2y3));  // 4
  likelihood(1, 2) = point_occupied_ * (1 - abs(Px3x4 * Py2y3));  // 5

  likelihood(2, 0) = point_occupied_ * (1 - abs(Px1x2 * Py1y2));  // 6
  likelihood(2, 1) = point_occupied_ * (1 - abs(Px2x3 * Py1y2));  // 7
  likelihood(2, 2) = point_occupied_ * (1 - abs(Px3x4 * Py1y2));  // 8
  ROS_INFO_COND(debug_ > 0, "Likelihood = \n%f %f %f\n%f %f %f\n%f %f % f", likelihood(0, 0), likelihood(0, 1),
                likelihood(0, 2), likelihood(1, 0), likelihood(1, 1), likelihood(1, 2), likelihood(2, 0),
                likelihood(2, 1), likelihood(2, 2));

  likelihood(0, 0) = 0.55;  // 0
  likelihood(0, 1) = 0.6;  // 1
  likelihood(0, 2) = 0.55;  // 2

  likelihood(1, 0) = 0.6;  // 3
  likelihood(1, 1) = 0.8;  // 4
  likelihood(1, 2) = 0.6;  // 5

  likelihood(2, 0) = 0.55;  // 6
  likelihood(2, 1) = 0.6;  // 7
  likelihood(2, 2) = 0.55;  // 8
*/
  likelihood(0, 0) = 0.5;  // 0
  likelihood(0, 1) = 0.5;  // 1
  likelihood(0, 2) = 0.5;  // 2

  likelihood(1, 0) = 0.5;  // 3
  likelihood(1, 1) = 0.8;  // 4
  likelihood(1, 2) = 0.5;  // 5

  likelihood(2, 0) = 0.5;  // 6
  likelihood(2, 1) = 0.5;  // 7
  likelihood(2, 2) = 0.5;  // 8
}

/**
 * @brief Gets points which belong to a contour from LaserScan data, taking into account max and min range values
 *
 * @param scan data from the laser scan.
 * @return Eigen::Matrix2Xf Matrix with extracted points
 */
Eigen::Matrix2Xf RamonSlam2D::getPointsFromScan(sensor_msgs::LaserScan scan)
{
  Eigen::Matrix2Xf scan_points = Eigen::Matrix2Xf::Constant(2, scan.ranges.size(), 0.0);
  Eigen::Matrix2Xf points_out = Eigen::Matrix2Xf::Constant(2, scan.ranges.size(), 0.0);
  int points_count = 0, points_aux = 0;
  std::vector<float> angles;
  float pointmod_reg, pointmod_next, pointmod_res;

  for (float angle = scan.angle_min; angle < scan.angle_max; angle += scan.angle_increment)
  {
    angles.push_back(angle);
  }

  scan_points(0, 0) = -(sin(angles[0]) * scan.ranges[0]);
  scan_points(1, 0) = cos(angles[0]) * scan.ranges[0];

  for (int i = 1; i < scan.ranges.size(); i++)
  {
    if (scan.ranges[i] < scan.range_max && scan.ranges[i] > scan.range_min)
    {
      // First, convert scan to point (x,y)
      scan_points(0, i) = -(sin(angles[i]) * scan.ranges[i]);
      scan_points(1, i) = cos(angles[i]) * scan.ranges[i];

      // Then, evaluate if the point is part of a contour
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
        // If there are at least X contiguous points, they are considered to belong to a valid contour.
        if ((points_aux - 1) > min_adjacent_points_)
        {
          // Move points index points_aux times, otherwise they will be ovewritten
          points_count += points_aux;
        }
        // Restart points_aux
        points_aux = 0;
      }
    }
  }
  points_out.conservativeResize(2, points_count);  // Truncate the output matrix
  return points_out;
}

/**
 * @brief Computes rigid body transformations of the lidar data and returns the one that best matches with the map
 * M(t-1)
 *
 * @param scan_in Input laser scanner
 * @param scan_out Laser scanner transformed
 */
void RamonSlam2D::getRigidBodyTransform(Eigen::Matrix2Xf scan_in, Eigen::Matrix2Xf& scan_out)
{
  Eigen::Matrix2Xf scan_aux = Eigen::Matrix2Xf::Constant(2, scan_in.cols(), 0.0);
  scan_out = Eigen::Matrix2Xf::Constant(2, scan_in.cols(), 0.0);
  float delta_theta = M_PI / 360;
  float delta_x = res_;
  float delta_y = res_;
  int i = 0, j = 0, k = 0, m;
  float x_out = x_, y_out = y_, theta_out = theta_;
  uint32_t index_x, index_y;
  float sum, sum_before;
  int first_time = 1;

  for (i = -3; i <= 3; i++)
  {
    for (j = -3; j <= 3; j++)
    {
      for (k = -3; k <= 3; k++)
      {
        scan_aux =
            this->rotateAndTranslate2D(scan_in, (x_ + i * delta_x), (y_ + j * delta_y), (theta_ + k * delta_theta));

        sum = 0.0;
        for (m = 0; m < scan_aux.cols(); m++)
        {
          index_x = uint32_t(round((m_ / 2 + scan_aux(0, m) + (x_ + i * delta_x)) / res_));
          index_y = uint32_t(round((n_ / 2 + scan_aux(1, m) + (y_ + j * delta_y)) / res_));
          sum += pow((1 - map_eig_(index_x, index_y)), 2);
        }

        if (first_time)
        {
          first_time = 0;
          sum_before = sum;
        }
        else if (sum <= sum_before)
        {
          scan_out = scan_aux;
          x_out = (x_ + float(i * delta_x));
          y_out = (y_ + float(j * delta_y));
          theta_out = (theta_ + float(k * delta_theta));
          sum_before = sum;
        }
      }
    }
  }
  x_ = x_out;
  y_ = y_out;
  theta_ = theta_out;
  ROS_INFO("(x_, y_, theta_) = (%f, %f, %f)", x_, y_, theta_);
}

/**
 * @brief Initializes the private variables of the class
 *
 */
void RamonSlam2D::init()
{
  laser_count_ = 0;
  transform_publish_period_ = 0.05;

  point_free_ = 0.3;
  point_noinfo_ = 0.5;
  point_occupied_ = 0.7;
  point_dis_threshold_ = 0.1;
  min_adjacent_points_ = 3;

  map_eig_ = Eigen::MatrixXf::Constant(uint32_t(round(m_ / res_)), uint32_t(round(n_ / res_)), point_noinfo_);
  ROS_INFO_COND(debug_ == 0, "Tengo el eigen cargado con %f", map_eig_(4, 4));

  std_x_ = 0.01;
  std_y_ = 0.01;

  x_ = 0.0;
  y_ = 0.0;
  theta_ = 0.0;

  // Define slam frames in case they were not given by user
  if (!nh_.getParam("base_frame", base_frame_))
    base_frame_ = "base_link";
  if (!nh_.getParam("map_frame", map_frame_))
    map_frame_ = "map";
  if (!nh_.getParam("imu_frame", imu_frame_))
    imu_frame_ = "imu";
  if (!nh_.getParam("tf_delay", tf_delay_))
    tf_delay_ = transform_publish_period_;
}

/**
 * @brief Calculates the inverse measurement model for a laser scanner. It identifies three regions. The first where no
 * information is available occurs outside of the scanning arc. The second where objects are likely to exist, at the end
 * of the range measurement within the arc. The third are where objects are unlikely to exist, within the arc but with
 * less distance than the range measurement.
 *
 * @param scan_points
 * @return Eigen::MatrixXf
 */
Eigen::MatrixXf RamonSlam2D::inverseScanner(Eigen::Matrix2Xf scan_points)
{
  // Init map with no info value
  Eigen::MatrixXf m = Eigen::MatrixXf::Constant(uint32_t(round(m_ / res_)), uint32_t(round(n_ / res_)), point_noinfo_);

  uint32_t index_x, index_y;
  std::vector<geometry_msgs::Point32> points_free;
  Eigen::Vector2f point_aux;

  for (int i = 0; i < scan_points.cols(); i++)
  {
    this->bresenhamLineAlgorithm(x_ / res_, y_ / res_, scan_points(0, i) / res_, scan_points(1, i) / res_, points_free);
    if (points_free.size() > 0)
    {
      for (int index = 0; index < points_free.size(); index++)
      {
        if (index != points_free.size() - 1)
        {
          index_x = uint32_t(round(0.5 * m_ / res_ + points_free[index].x + x_ / res_));
          index_y = uint32_t(round(0.5 * n_ / res_ + points_free[index].y + y_ / res_));
          m(index_x, index_y) =
              this->getProbaFromLogit(this->getLogitFromProba(point_free_) +
                                      this->getLogitFromProba(m(index_x, index_y)) - getLogitFromProba(point_noinfo_));
        }
      }
    }
    index_x = uint32_t(round(0.5 * m_ / res_ + scan_points(0, i) / res_ + x_ / res_));
    index_y = uint32_t(round(0.5 * n_ / res_ + scan_points(1, i) / res_ + y_ / res_));

    point_aux(0) = scan_points(0, i) + x_;
    point_aux(1) = scan_points(1, i) + y_;
    this->logitUpdate(m, index_x, index_y, point_aux);
    points_free.clear();
  }
  return m;
}

/**
 * @brief Callback function of the laser scan topic
 *
 * @param scanptr Pointer to laser scan data
 */
void RamonSlam2D::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scanptr)
{
  sensor_msgs::LaserScan scan = *scanptr;
  float position_x, position_y;
  std::vector<float> angles;
  float theta;
  static int first_time = 1;
  nav_msgs::OccupancyGrid occmap;

  Eigen::MatrixXf map_scan;
  geometry_msgs::Point32 delta_point;
  Eigen::Matrix2Xf scan_points, scan_out;
  float delta_phi;

  uint32_t point_count;
  std::vector<geometry_msgs::Point32> points_aux;

  float x_out, y_out, theta_out;

  // Para calcular tiempos del algoritmo
  ros::WallTime start_, end_;

  if (first_time)
  {
    first_time = 0;
    return;
  }
  start_ = ros::WallTime::now();

  occmap.info.width = n_ / res_;
  occmap.info.height = m_ / res_;
  occmap.info.resolution = res_;
  occmap.info.origin.position.x = -0.5 * m_;
  occmap.info.origin.position.y = -0.5 * n_;
  occmap.info.origin.position.z = 0;
  for (int i = 0; i < (m_ * n_) / (res_ * res_); i++)
    occmap.data.push_back(-1);

  position_x = 0.0;
  position_y = 0.0;
  theta = 0.0;

  scan_points = this->getPointsFromScan(scan);

  laser_count_++;

  if (laser_count_ > 1)
  {
    this->getRigidBodyTransform(scan_points, scan_points);
  }
  else
  {
    scan_points = this->rotateAndTranslate2D(scan_points, 0.0, 0.0, 0.0);
  }

  map_scan = this->inverseScanner(scan_points);

  for (int x = 0; x < map_scan.rows(); x++)
  {
    for (int y = 0; y < map_scan.cols(); y++)
    {
      // Update stored map with current measurement
      map_eig_(x, y) =
          this->getProbaFromLogit(this->getLogitFromProba(map_scan(x, y)) + this->getLogitFromProba(map_eig_(x, y)) -
                                  getLogitFromProba(point_noinfo_));

      if (map_eig_(x, y) != point_noinfo_)
        occmap.data[MAP_IDX(occmap.info.width, x, y)] = uint8_t(map_eig_(x, y) * 100);
      else
        occmap.data[MAP_IDX(occmap.info.width, x, y)] = -1;
    }
  }

  end_ = ros::WallTime::now();
  double execution_time = (end_ - start_).toNSec() * 1e-6;
  ROS_INFO_STREAM_COND(debug_ < 0, "Exectution time (ms): " << execution_time);
  map_pub_.publish(occmap);
}

/**
 * @brief Updates the values of the probability map given the current scan, based on the logit function. Spreads the
 * point up to 2 adjoining cells (3x3 cell) taking Gaussian distribution.
 *
 * @param m Lidar map of current scan
 * @param index_x Index x of the point in the map
 * @param index_y Index y of the point in the map
 * @param point Point value
 */
void RamonSlam2D::logitUpdate(Eigen::MatrixXf& m, uint32_t index_x, uint32_t index_y, Eigen::Vector2f point)
{
  float logit_t0 = getLogitFromProba(point_noinfo_);
  Eigen::Matrix3f likelihood;
  this->getOccupancyLikelihood(likelihood, point);
  for (int x = 0; x <= 2; x++)
  {
    for (int y = 0; y <= 2; y++)
    {
      if ((index_x + x - 1) > 0 && (index_y + y - 1) > 0)
      {
        m(index_x + x - 1, index_y + y - 1) =
            this->getProbaFromLogit(this->getLogitFromProba(likelihood(x, y)) +
                                    this->getLogitFromProba(m(index_x + x - 1, index_y + y - 1)) - logit_t0);
        ROS_INFO_COND(debug_ > 0, "En m(%d,%d)=%f", index_x + x - 1, index_y + x - 1,
                      m(index_x + x - 1, index_y + y - 1));
      }
    }
  }
}

/**
 * @brief Publishes the transformed point of the vehicle
 *
 */
void RamonSlam2D::publishTransform(void)
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