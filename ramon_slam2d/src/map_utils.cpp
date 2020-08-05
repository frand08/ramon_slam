/**
 * @file map_utils.cpp
 * @author Francisco A. Dominguez (dominguezfranciscoa@gmail.com)
 * @brief
 * @version 0.1
 * @date 2020-01-09
 *
 * @copyright Copyright (c) 2020
 *
 */

#include "map_utils.h"

/* Public Functions */

/**
 * @brief Construct a new MapUtils::MapUtils object
 *
 */
MapUtils::MapUtils()
{

}

/**
 * @brief Destroy the MapUtils::MapUtils object
 *
 */
MapUtils::~MapUtils()
{

}

/**
 * @brief
 *
 * @param x0
 * @param y0
 * @param x1
 * @param y1
 * @param res
 * @param point
 */
int MapUtils::bresenhamLineAlgorithm(double x0, double y0, double x1, double y1,
                                         std::vector<geometry_msgs::Point32>& point)
{
  int ret = -1;
  if (abs(y1 - y0) < abs(x1 - x0))
  {
    if (x0 > x1)
    {
      ret = 0;
      this->bresenhamLineLow(x1, y1, x0, y0, 1, point);
    }
    else
    {
      ret = 1;
      this->bresenhamLineLow(x0, y0, x1, y1, 0, point);
    }
  }
  else
  {
    if (y0 > y1)
    {
      ret = 2;
      this->bresenhamLineHigh(x1, y1, x0, y0, 1, point);
    }
    else
    {
      ret = 3;
      this->bresenhamLineHigh(x0, y0, x1, y1, 0, point);
    }
  }
  return ret;
}

/**
 * @brief Obtains the Gaussian Blur Integral
 *
 * @param x0
 * @param x1
 * @param std_dev
 * @return double
 */
double MapUtils::gaussianBlur1D(double x0, double x1, double std_dev)
{
  // Qian2019 - P.7
  // erf: error function
  return (1 / (2 * M_PI * std_dev) * exp(-pow(x0 - x1,2) / (2 * std_dev * std_dev)));
}

/**
 * @brief Obtains the Gaussian Blur Integral
 *
 * @param a
 * @param b
 * @param c
 * @param std
 * @return double
 */
double MapUtils::gaussianBlurIntegral(double a, double b, double c, double std)
{
  // Qian2019 - P.7
  // erf: error function
  return (-0.5 * (erf((c - b) / (sqrt(2) * std)) - erf((c - a) / (sqrt(2) * std))));
}

/**
 * @brief Gets the logit of the probability p, ie, logit(p) = log(p / (1 - p))
 *
 * @param value Probability value
 * @return double logit(value)
 */
double MapUtils::getLogitFromProba(double value)
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
 * @return double
 */
double MapUtils::getProbaFromLogit(double value)
{
  return (std::exp(value) / (1 + std::exp(value)));
}

/* Private Functions */


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
void MapUtils::bresenhamLineHigh(double x0, double y0, double x1, double y1, int direction,
                                    std::vector<geometry_msgs::Point32>& point)
{
  double dx = x1 - x0;
  double dy = y1 - y0;
  double xi = 1;
  double D;
  double y, x;
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
void MapUtils::bresenhamLineLow(double x0, double y0, double x1, double y1, int direction,
                                   std::vector<geometry_msgs::Point32>& point)
{
  double dx = x1 - x0;
  double dy = y1 - y0;
  double yi = 1;
  double D;
  double y, x;
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
 * @brief Updates the values of the probability map given the current scan point, based on the logit function. Spreads
 * the point up to 2 adjoining cells (3x3 cell) taking Gaussian distribution.
 *
 * @param m Lidar map of current scan
 * @param index_x Index x of the point in the map
 * @param index_y Index y of the point in the map
 * @param point Point value
 */
void MapUtils::logitUpdate(Eigen::Ref<Eigen::MatrixXd> m, Eigen::Vector2i index, Eigen::Vector2d point, Eigen::Vector2d std_dev, double res)
{
  double logit_t0 = this->getLogitFromProba(point_noinfo_);
  Eigen::Matrix3d likelihood;
  this->getOccupancyLikelihood(likelihood, point, std_dev, res);

  for (int x = 0; x < likelihood.cols(); x++)
  {
    for (int y = 0; y < likelihood.rows(); y++)
    {
      if (m.rows() >= (index(0) + x) && m.cols() >= (index(1) + y) && (index(0) + x - 1) > 0 && (index(1) + y - 1) > 0)
      {
        m(index(0) + x - 1, index(1) + y - 1) =
               1 - (1 - m(index(0) + x - 1, index(1) + y - 1)) * likelihood(x, y);
        // m(index(0) + x - 1, index(1) + y - 1) =
        //     this->getProbaFromLogit(this->getLogitFromProba(likelihood(x, y)) +
        //                             this->getLogitFromProba(m(index(0) + x - 1, index(1) + y - 1)) - logit_t0);
      }
    }
  }
}



/**
 * @brief Gets the occupancy likelihood around a point (3x3 matrix). Gaussian assumption
 *
 * @param likelihood 3x3 matrix return data
 * @param point Point location
 * @param std_dev Standard deviation of point
 * @param res Map resolution
 */
void MapUtils::getOccupancyLikelihood(Eigen::Matrix3d& likelihood, Eigen::Vector2d point, Eigen::Vector2d std_dev, double res)
{
  // x likelihood values
  double Px1x2, Px2x3, Px3x4;
  // y likelihood values
  double Py1y2, Py2y3, Py3y4;

  // x positions of square lines
  double x1, x2, x3, x4;
  // y positions of square lines
  double y1, y2, y3, y4;

  x2 = point(0);
  x1 = x2 - res;
  x3 = x2 + res;
  x4 = x3 + res;

  y2 = point(1);
  y1 = y2 - res;
  y3 = y2 + res;
  y4 = y3 + res;

  // Grid Point Occupation - 9 grid cells around the point using Gaussian assumption
  // x-coordinate occupancy likelihoods (Gaussian assumption)
  Px1x2 = this->gaussianBlurIntegral(x1, x2, point(0), std_dev(0));
  Px2x3 = this->gaussianBlurIntegral(x2, x3, point(0), std_dev(0));
  Px3x4 = this->gaussianBlurIntegral(x3, x4, point(0), std_dev(0));

  // y-coordinate occupancy likelihoods (Gaussian assumption)
  Py1y2 = this->gaussianBlurIntegral(y1, y2, point(1), std_dev(1));
  Py2y3 = this->gaussianBlurIntegral(y2, y3, point(1), std_dev(1));
  Py3y4 = this->gaussianBlurIntegral(y3, y4, point(1), std_dev(1));

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

  // likelihood(0, 0) = (abs((1/(2*M_PI*std_dev(0)) * exp(-pow(res,2) / (2*std_dev(0)*std_dev(0)))) *
  //                    (1/(2*M_PI*std_dev(1)) * exp(-pow(res,2) / (2*std_dev(1)*std_dev(1))))
  //                    ));
  // likelihood(0,2) = likelihood(0,0);
  // likelihood(2,0) = likelihood(0,0);
  // likelihood(2,2) = likelihood(0,0);
  // likelihood(1,0) = (abs((1/(2*M_PI*std_dev(0)) * exp(-pow(0,2) / (2*std_dev(0)*std_dev(0)))) *
  //                    (1/(2*M_PI*std_dev(1)) * exp(-pow(res,2) / (2*std_dev(1)*std_dev(1))))
  //                    ));
  // likelihood(1,2) = likelihood(1,0);
  // likelihood(0,1) = (abs((1/(2*M_PI*std_dev(0)) * exp(-pow(res,2) / (2*std_dev(0)*std_dev(0)))) *
  //                    (1/(2*M_PI*std_dev(1)) * exp(-pow(0,2) / (2*std_dev(1)*std_dev(1))))
  //                    ));
  // likelihood(2,1) = likelihood(0,1);
  // likelihood(1,1) = point_occupied_;
}


/**
 * @brief Rotates and translates a 2xN matrix
 *
 * @param matrix Matrix to be translated and rotated
 * @param x translation of x
 * @param y translation of y
 * @param theta rotation in radians
 * @return Eigen::Matrix2Xd
 */
Eigen::Matrix2Xd MapUtils::rotateAndTranslate2D(Eigen::Matrix2Xd matrix, double x, double y, double theta)
{
  // This class is equivalent to a single scalar representing a counter clock wise rotation as a single angle in radian.
  Eigen::Rotation2Dd rot2(theta);
  Eigen::Matrix2Xd matrix_out;

  // First, do rotation
  if(theta != 0.0)
    matrix_out = rot2.toRotationMatrix() * matrix;
  else
    matrix_out = matrix;
  
  // Then translation
  matrix_out.row(0) = matrix_out.row(0) + Eigen::MatrixXd::Constant(1, matrix_out.cols(), x);
  matrix_out.row(1) = matrix_out.row(1) + Eigen::MatrixXd::Constant(1, matrix_out.cols(), y);

  return matrix_out;
}
