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
  res_ = 0.02;
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
SLAM2D::SLAM2D(uint32_t M, uint32_t N, float res) : transform_thread_(NULL), debug_(DEBUG_MODE), nh_("~")
{
  m_ = M;
  n_ = N;
  res_ = res;
  res_low_ = 10 * res;
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
 * @brief
 *
 * @param x0
 * @param y0
 * @param x1
 * @param y1
 * @param point
 */
void SLAM2D::bresenhamLineAlgorithm(float x0, float y0, float x1, float y1,
                                         std::vector<geometry_msgs::Point32>& point)
{
  if (abs(y1 - y0) < abs(x1 - x0))
  {
    if (x0 > x1)
    {
      this->bresenhamLineLow(x1, y1, x0, y0, 1, point);
    }
    else
    {
      this->bresenhamLineLow(x0, y0, x1, y1, 0, point);
    }
  }
  else
  {
    if (y0 > y1)
    {
      this->bresenhamLineHigh(x1, y1, x0, y0, 1, point);
    }
    else
    {
      this->bresenhamLineHigh(x0, y0, x1, y1, 0, point);
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
float SLAM2D::gaussianBlurIntegral(float a, float b, float c, float std)
{
  // Qian2019 - P.7
  // erf: error function
  return (-0.5 * (erf((c - b) / (sqrt(2) * std)) - erf((c - a) / (sqrt(2) * std))));
}

/**
 * @brief Gets the logit of the probability p, ie, logit(p) = log(p / (1 - p))
 *
 * @param value Probability value
 * @return float logit(value)
 */
float SLAM2D::getLogitFromProba(float value)
{
  // Avoid infinite values
  if (value > 0.99)
    return 4.6;
  else if (value < 0.01)
    return -4.6;
  else
    return log(value / (1 - value));
}

/**
 * @brief Gets the probability p from the logarithm of the odds p / (1 - p)
 *
 * @param value
 * @return float
 */
float SLAM2D::getProbaFromLogit(float value)
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
Eigen::Matrix2Xf SLAM2D::rotateAndTranslate2D(Eigen::Matrix2Xf matrix, float x, float y, float theta)
{
  // This class is equivalent to a single scalar representing a counter clock wise rotation as a single angle in radian.
  Eigen::Rotation2Df rot2(theta);
  Eigen::Matrix2Xf matrix_out;

  // First, do rotation
  matrix_out = rot2.toRotationMatrix() * matrix;

  // Then translation
  matrix_out.row(0) = matrix_out.row(0) + Eigen::MatrixXf::Constant(1, matrix_out.cols(), x);
  matrix_out.row(1) = matrix_out.row(1) + Eigen::MatrixXf::Constant(1, matrix_out.cols(), y);

  return matrix_out;
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

  // TF thread
  transform_thread_ = new boost::thread(boost::bind(&SLAM2D::publishTransform, this));
}

/* Private functions */

/**
 * @brief
 *
 * @param x0
 * @param y0
 * @param x1
 * @param y1
 * @param direction
 * @param point
 */
void SLAM2D::bresenhamLineHigh(float x0, float y0, float x1, float y1, int direction,
                                    std::vector<geometry_msgs::Point32>& point)
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
  if (direction)
    std::reverse(point.begin(), point.end());
}

/**
 * @brief
 *
 * @param x0
 * @param y0
 * @param x1
 * @param y1
 * @param direction
 * @param point
 */
void SLAM2D::bresenhamLineLow(float x0, float y0, float x1, float y1, int direction,
                                   std::vector<geometry_msgs::Point32>& point)
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
  if (direction)
    std::reverse(point.begin(), point.end());
}

/**
 * @brief Gets the maximum likelihood transform for a given delta x
 *
 * @param x_index delta x
 * @param rigid Information about the input scan, offsets and output scans (reference)
 * @return int zero value
 */
int SLAM2D::getMaximumLikelihoodTransform(int x_index, rigid_t& rigid)
{
  float delta_x = res_;
  float delta_y = res_;
  float delta_theta = M_PI / 360;
  float sum;
  int index_x, index_y;
  Eigen::Matrix2Xf scan_aux;

  for (int y_index = -3; y_index <= 3; y_index++)
  {
    for (int theta_index = -3; theta_index <= 3; theta_index++)
    {
      scan_aux = this->rotateAndTranslate2D(rigid.scan_in, (x_ + x_index * delta_x), (y_ + y_index * delta_y),
                                            (theta_ + theta_index * delta_theta));

      sum = 0.0;
      for (int m = 0; m < scan_aux.cols(); m++)
      {
        index_x = int(round((m_ / 2 + scan_aux(0, m)) / res_));
        index_y = int(round((n_ / 2 + scan_aux(1, m)) / res_));
        sum += pow((1 - map_eig_(index_x, index_y)), 2);
      }

      laser_processing_mutex_.lock();
      if (sum < rigid.sum_out)
      {
        rigid.scan_out = scan_aux;
        rigid.x_out = x_index * delta_x;
        rigid.y_out = y_index * delta_y;
        rigid.theta_out = theta_index * delta_theta;
        rigid.sum_out = sum;
      }
      laser_processing_mutex_.unlock();
    }
  }
  return 0;
}

/**
 * @brief Gets the occupancy likelihood around a point (3x3 matrix). Gaussian assumption
 *
 * @param likelihood 3x3 matrix return data
 * @param point Point location
 */
void SLAM2D::getOccupancyLikelihood(Eigen::Matrix3f& likelihood, Eigen::Vector2f point)
{
  // x likelihood values
  float Px1x2, Px2x3, Px3x4;
  // y likelihood values
  float Py1y2, Py2y3, Py3y4;

  // x positions of square lines
  float x1, x2, x3, x4;
  // y positions of square lines
  float y1, y2, y3, y4;

  x2 = point(0);
  x1 = x2 - res_;
  x3 = x2 + res_;
  x4 = x3 + res_;

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

  /* FIXME: Px and Py gave wrong values? */
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
}

/**
 * @brief Gets points which belong to a contour from LaserScan data, taking into account max and min range values
 *
 * @param scan data from the laser scan.
 * @param points_out Matrix with extracted points
 * @return int 0: ok
 */
int SLAM2D::getPointsFromScan(sensor_msgs::LaserScan scan, Eigen::Matrix2Xf &points_out)
{
  Eigen::Matrix2Xf scan_points = Eigen::Matrix2Xf::Constant(2, scan.ranges.size(), 0.0);
  int points_count = 0, points_aux = 0, i;
  std::vector<float> angles;
  float pointmod_reg, pointmod_next, pointmod_res;
  bool first_time = true;
  
  points_out = Eigen::Matrix2Xf::Constant(2, scan.ranges.size(), 0.0);
  
  for (float angle = scan.angle_min; angle < scan.angle_max; angle += scan.angle_increment)
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
void SLAM2D::getRigidBodyTransform(const Eigen::Ref<const Eigen::Matrix2Xf> scan_in, Eigen::Matrix2Xf &scan_out)
{
  float x_out, y_out, theta_out;
  float sum_out;
  int i;
  rigid_t rigid;
  std::vector<boost::thread*> rigid_body_threads;
  rigid.scan_in = scan_in;
  rigid.sum_out = 10000.0;
  for (i = -3; i <= 3; i++)
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

  // Update pose data and output scan
  x_ = x_ + rigid.x_out;
  y_ = y_ + rigid.y_out;
  theta_ = theta_ + rigid.theta_out;
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
int SLAM2D::getRigidBodyTransform(const Eigen::Ref<const Eigen::Matrix2Xf> scan_in, Eigen::Matrix2Xf &scan_out, int innecesario)
{
  float x_out, y_out, theta_out;
  float sum_out;
  int i;
	Eigen::Vector2f center_src, center_dst;

  center_src = scan_in.rowwise().mean();
  center_dst = map_points_.rowwise().mean();

  Eigen::MatrixXf Dt = (map_points_ - center_dst).transpose();
  Eigen::Matrix2f H = Dt * (scan_in - center_src);
  // Eigen::Matrix2f W, U, V;

  Eigen::JacobiSVD<Eigen::MatrixXf> svd;
  Eigen::MatrixXf H_(2,2);

  H_ << H;

  svd.compute(H_, Eigen::ComputeThinU | Eigen::ComputeThinV);
  if (!svd.computeU() || !svd.computeV()) 
  {
    ROS_ERROR("decomposition error");
		return -1;
	}

  Eigen::Matrix2f Vt = svd.matrixV().transpose();
  Eigen::Matrix2f R = svd.matrixU() * Vt;
  Eigen::Vector2f t = center_dst - R * center_src;
  float theta = atan2(R(0,1), R(0,0));

  // Update pose data and output scan
  x_ = x_ + t(0);
  y_ = y_ + t(1);
  theta_ = theta_ + theta;
  scan_out = rotateAndTranslate2D(scan_in, t(0), t(1), theta);

  ROS_INFO("(x_, y_, theta_) = (%f, %f, %f)", x_, y_, theta_);
  return 0;
}

/**
 * @brief Computes rigid body transformations of the lidar data and returns the one that best matches with the map
 * M(t-1)
 * https://gist.github.com/JiaxiangZheng/8168862
 * @param scan_in Input laser scanner
 * @param scan_out Laser scanner transformed
 */
int SLAM2D::computeRigidTransform(const Eigen::Ref<const Eigen::Matrix2Xf> scan_in, Eigen::Matrix2Xf &scan_out)
{
  float x_out, y_out, theta_out;
  float sum_out;
  int i;
	Eigen::Vector2f center_src, center_dst;

  center_src = scan_in.rowwise().mean();
  center_dst = map_points_.rowwise().mean();

  Eigen::MatrixXf Dt = (map_points_ - center_dst).transpose();
  Eigen::Matrix2f H = Dt * (scan_in - center_src);
  // Eigen::Matrix2f W, U, V;

  Eigen::JacobiSVD<Eigen::MatrixXf> svd;
  Eigen::MatrixXf H_(2,2);

  H_ << H;

  svd.compute(H_, Eigen::ComputeThinU | Eigen::ComputeThinV);
  if (!svd.computeU() || !svd.computeV()) 
  {
    ROS_ERROR("decomposition error");
		return -1;
	}

  Eigen::Matrix2f Vt = svd.matrixV().transpose();
  Eigen::Matrix2f R = svd.matrixU() * Vt;
  Eigen::Vector2f t = center_dst - R * center_src;
  float theta = atan2(R(0,1), R(0,0));

  // Update pose data and output scan
  x_ = x_ + t(0);
  y_ = y_ + t(1);
  theta_ = theta_ + theta;
  scan_out = rotateAndTranslate2D(scan_in, t(0), t(1), theta);

  ROS_INFO("(x_, y_, theta_) = (%f, %f, %f)", x_, y_, theta_);
  return 0;
}

/**
 * @brief Computes rigid body transformations of the lidar data and returns the one that best matches with the map
 * M(t-1)
 *
 * @param scan_in Input laser scanner
 * @param scan_out Laser scanner transformed
 */
/*
int SLAM2D::computeRigidTransform(const Eigen::Ref<const Eigen::MatrixXf> map_scan)
{
  float x_out, y_out, theta_out;
  float sum_out;
  int i;
	Eigen::Vector2f center_src, center_dst;

  center_src = map_scan.rowwise().mean();
  center_dst = map_eig_.rowwise().mean();


  Eigen::MatrixXf Dt = (map_eig_ - center_dst).transpose();
  Eigen::Matrix2f H = Dt * (map_scan - center_src);
  // Eigen::Matrix2f W, U, V;

  Eigen::JacobiSVD<Eigen::MatrixXf> svd;
  Eigen::MatrixXf H_(2,2);

  H_ << H;

  svd.compute(H_, Eigen::ComputeThinU | Eigen::ComputeThinV);
  if (!svd.computeU() || !svd.computeV()) 
  {
    ROS_ERROR("decomposition error");
		return -1;
	}

  Eigen::Matrix2f Vt = svd.matrixV().transpose();
  Eigen::Matrix2f R = svd.matrixU() * Vt;
  Eigen::Vector2f t = center_dst - R * center_src;
  float theta = atan2(R(0,1), R(0,0));

  // Update pose data and output scan
  x_ = x_ + t(0);
  y_ = y_ + t(1);
  theta_ = theta_ + theta;
  scan_out = rotateAndTranslate2D(scan_in, t(0), t(1), theta);

  ROS_INFO("(x_, y_, theta_) = (%f, %f, %f)", x_, y_, theta_);
  return 0;
}
*/
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
  min_adjacent_points_ = 3;

  map_eig_ = Eigen::MatrixXf::Constant(uint32_t(round(m_ / res_)), uint32_t(round(n_ / res_)), point_noinfo_);
  map_eig_low_ = Eigen::MatrixXf::Constant(uint32_t(round(m_ / res_low_)), uint32_t(round(n_ / res_low_)), point_noinfo_);

  std_x_ = 0.01;
  std_y_ = 0.01;

  x_ = 0.0;
  y_ = 0.0;
  theta_ = 0.0;

  // Init occupancygrid map
  occmap_.info.width = n_ / res_;
  occmap_.info.height = m_ / res_;
  occmap_.info.resolution = res_;
  occmap_.info.origin.position.x = -0.5 * m_;
  occmap_.info.origin.position.y = -0.5 * n_;
  occmap_.info.origin.position.z = 0;
  for (int i = 0; i < (m_ * n_) / (res_ * res_); i++)
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

  ROS_INFO_STREAM("SCAN: " << scan_topic_name_);
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
void SLAM2D::inverseScanner(Eigen::Matrix2Xf scan_points, Eigen::MatrixXf &m, Eigen::MatrixXf &m_low)
{
  /* TODO: Da como el orto, se empieza a correr, creo que hay quilombo con x_, y_ y todo eso, y con solo sumarle res_ al
   * map_size deberia alcanzar, pero no lo hace, que se yo */

  // Index of each point in map
  uint32_t index_x, index_y, index_x_low, index_y_low;
  // Points free between the vehicle and the measured obstacle
  std::vector<geometry_msgs::Point32> points_free, points_free_low;
  // Point to be updated by the logit function in each step
  Eigen::Vector2f point_update;
  // Max and min values of the scan point vector
  Eigen::Vector2f max_val_rows = scan_points.rowwise().maxCoeff();
  Eigen::Vector2f min_val_rows = scan_points.rowwise().minCoeff();

  // Map min size in x and y
  Eigen::Vector2f map_size, map_size_low;

  /* FIXME: why is res_ not enough? */
  // Get the max value of rows and cols
  if (std::abs(max_val_rows(0)) >= std::abs(min_val_rows(0)))
  {
    map_size(0) = std::abs(max_val_rows(0)) + 10 * res_;
    map_size_low(0) = std::abs(max_val_rows(0)) + 10 * res_low_;
  }
  else
  {
    map_size(0) = std::abs(min_val_rows(0)) + 10 * res_;
    map_size_low(0) = std::abs(min_val_rows(0)) + 10 * res_low_;
  }

  if (std::abs(max_val_rows(1)) >= std::abs(min_val_rows(1)))
  {
    map_size(1) = std::abs(max_val_rows(1)) + 10 * res_;
    map_size_low(1) = std::abs(max_val_rows(1)) + 10 * res_low_;
  }
  else
  {
    map_size(1) = std::abs(min_val_rows(1)) + 10 * res_;
    map_size_low(1) = std::abs(min_val_rows(1)) + 10 * res_low_;
  }
  ROS_INFO("ASDASDAS");
  // Init map with no info value
  m = Eigen::MatrixXf::Constant(uint32_t(round(2 * map_size(0) / res_)), uint32_t(round(2 * map_size(1) / res_)),
                                point_noinfo_);
  m_low = Eigen::MatrixXf::Constant(uint32_t(round(2 * map_size_low(0) / res_low_)), uint32_t(round(2 * map_size_low(1) / res_low_)),
                                point_noinfo_);
  ROS_INFO("SEEEEEEE");
  for (int i = 0; i < scan_points.cols(); i++)
  {
    // Get the free points between the vehicle and each laser point
    this->bresenhamLineAlgorithm(0, 0, scan_points(0, i) / res_, scan_points(1, i) / res_, points_free);

    if (points_free.size() > 0)
    {
      for (int index = 0; index < points_free.size() - 1; index++)
      {
        // Update free points in map
        index_x = uint32_t(round(round(map_size(0) / res_) + points_free[index].x));
        index_y = uint32_t(round(round(map_size(1) / res_) + points_free[index].y));

        m(index_x, index_y) =
            this->getProbaFromLogit(this->getLogitFromProba(point_free_) +
                                    this->getLogitFromProba(m(index_x, index_y)) - getLogitFromProba(point_noinfo_));
      }
    }
    index_x = uint32_t(round(round(map_size(0) / res_) + scan_points(0, i) / res_));
    index_y = uint32_t(round(round(map_size(1) / res_) + scan_points(1, i) / res_));

    // Update occupied points in map
    
    // Update around point option (3x3 matrix)
    // point_update(0) = scan_points(0, i) / res_;
    // point_update(1) = scan_points(1, i) / res_;
    // this->logitUpdate(m, index_x, index_y, point_update);

    // One point update option
    m(index_x, index_y) =
        this->getProbaFromLogit(this->getLogitFromProba(point_occupied_) +
                                this->getLogitFromProba(m(index_x, index_y)) - getLogitFromProba(point_noinfo_));

    // Clear free points vector
    points_free.clear();



    // Low map resolution
    this->bresenhamLineAlgorithm(0, 0, scan_points(0, i) / res_low_, scan_points(1, i) / res_low_, points_free_low);

    if (points_free_low.size() > 0)
    {
      for (int index = 0; index < points_free_low.size() - 1; index++)
      {
        // Update free points in map
        index_x_low = uint32_t(round(round(map_size_low(0) / res_low_) + points_free_low[index].x));
        index_y_low = uint32_t(round(round(map_size_low(1) / res_low_) + points_free_low[index].y));

        m_low(index_x_low, index_y_low) =
            this->getProbaFromLogit(this->getLogitFromProba(point_free_) +
                                    this->getLogitFromProba(m_low(index_x_low, index_y_low)) - getLogitFromProba(point_noinfo_));
      }
    }
    index_x_low = uint32_t(round(round(map_size_low(0) / res_low_) + scan_points(0, i) / res_low_));
    index_y_low = uint32_t(round(round(map_size_low(1) / res_low_) + scan_points(1, i) / res_low_));
    
    m_low(index_x_low, index_y_low) =
        this->getProbaFromLogit(this->getLogitFromProba(point_occupied_) +
                                this->getLogitFromProba(m_low(index_x_low, index_y_low)) - getLogitFromProba(point_noinfo_));

    points_free_low.clear();
  }
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

  Eigen::MatrixXf map_scan, map_scan_low;
  Eigen::Matrix2Xf scan_points, scan_out;

  // Para calcular tiempos del algoritmo
  ros::WallTime start_, end_;

  if (first_time)
  {
    first_time = 0;
    return;
  }

  start_ = ros::WallTime::now();

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
    this->getRigidBodyTransform(scan_points, scan_out);
  }
  else
  {
    scan_out = scan_points;
  }

  last = now;

  // Get map points from scan
  this->inverseScanner(scan_out, map_scan, map_scan_low);

  this->mapUpdate(map_scan);

  end_ = ros::WallTime::now();
  double execution_time = (end_ - start_).toNSec() * 1e-6;
  ROS_INFO_STREAM_COND(debug_ < 0, "Exectution time (ms): " << execution_time);
  map_pub_.publish(occmap_);
}

/**
 * @brief Updates saved map
 *
 * @param map_scan Lidar map of transformed scan
 */
void SLAM2D::mapUpdate(Eigen::MatrixXf map_scan)
{
  int map_delta_x;
  int map_delta_y;

  map_delta_x = int(round(0.5 * (m_ / res_ - map_scan.rows())));
  map_delta_y = int(round(0.5 * (n_ / res_ - map_scan.cols())));

  for (int x = 0; x < map_scan.rows(); x++)
  {
    for (int y = 0; y < map_scan.cols(); y++)
    {
      // Update stored map with current measurement
      map_eig_(x + map_delta_x, y + map_delta_y) = this->getProbaFromLogit(
          this->getLogitFromProba(map_scan(x, y)) +
          this->getLogitFromProba(map_eig_(x + map_delta_x, y + map_delta_y)) - this->getLogitFromProba(point_noinfo_));

      occmap_.data[MAP_IDX(occmap_.info.width, x + map_delta_x, y + map_delta_y)] =
          int8_t(map_eig_(x + map_delta_x, y + map_delta_y) * 100);
    }
  }
}

/**
 * @brief Updates the values of the probability map given the current scan point, based on the logit function. Spreads
 * the point up to 2 adjoining cells (3x3 cell) taking Gaussian distribution.
 *
 * @param m Lidar map of current scan
 * @param index_x Index x of the point in the map
 * @param index_y Index y of the point in the map
 * @param point Point value
 */
void SLAM2D::logitUpdate(Eigen::MatrixXf& m, uint32_t index_x, uint32_t index_y, Eigen::Vector2f point)
{
  float logit_t0 = getLogitFromProba(point_noinfo_);
  Eigen::Matrix3f likelihood;
  this->getOccupancyLikelihood(likelihood, point);

  for (int x = 0; x < likelihood.cols(); x++)
  {
    for (int y = 0; y < likelihood.rows(); y++)
    {
      if (m.rows() >= (index_x + x) && m.cols() >= (index_y + y) && (index_x + x - 1) > 0 && (index_y + y - 1) > 0)
      {
        m(index_x + x - 1, index_y + y - 1) =
            this->getProbaFromLogit(this->getLogitFromProba(likelihood(x, y)) +
                                    this->getLogitFromProba(m(index_x + x - 1, index_y + y - 1)) - logit_t0);
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
