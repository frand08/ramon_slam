/**
 * @file map_utils.h
 * @author Francisco A. Dominguez (dominguezfranciscoa@gmail.com)
 * @brief
 * @version 0.1
 * @date 2020-01-09
 *
 * @copyright Copyright (c) 2020
 *
 */

#ifndef _map_utils_h
#define _map_utils_h

#include <geometry_msgs/Point32.h>
#include <Eigen/Dense>

	/**
	* @class MapUtils
	* @brief Clase para representar celdas.
	*/
  class MapUtils
  {
  public:
    MapUtils();
    ~MapUtils();

    void bresenhamLineAlgorithm(double x0, double y0, double x1, double y1, std::vector<geometry_msgs::Point32>& point);
    double gaussianBlurIntegral(double a, double b, double c, double std);
      
    void getOccupancyLikelihood(Eigen::Matrix3d& likelihood, Eigen::Vector2d point, Eigen::Vector2d std_dev, double res);

    double getLogitFromProba(double value);
    double getProbaFromLogit(double value);
    void logitUpdate(Eigen::MatrixXd& m, Eigen::Vector2i index, Eigen::Vector2d point, Eigen::Vector2d std_dev, double res);

    Eigen::Matrix2Xd rotateAndTranslate2D(Eigen::Matrix2Xd matrix, double x, double y, double theta);

  protected:
    double point_free_, point_noinfo_, point_occupied_;

  private:
    // https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
    void bresenhamLineHigh(double x0, double y0, double x1, double y1, int direction,
                          std::vector<geometry_msgs::Point32>& point);
    void bresenhamLineLow(double x0, double y0, double x1, double y1, int direction,
                          std::vector<geometry_msgs::Point32>& point);

  };

#endif //_map_utils_h