/**
 * @file imu_calibration.h
 * @author Francisco A. Dominguez (dominguezfranciscoa@gmail.com)
 * @brief
 * @version 0.1
 * @date 2020-06-20
 *
 * @copyright Copyright (c) 2020
 *
 */

#ifndef _IMU_CALIBRATION_H
#define _IMU_CALIBRATION_H

#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <ramon_msgs/ImuWithMag.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <Eigen/Eigen>
#include <unsupported/Eigen/NonLinearOptimization>

#include "gyro_calibration.h"
#include "accel_calibration.h"
#include "mag_calibration.h"

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#define GRAVITY -9.80665f

namespace ramon_calibration
{
	/**
	* @class AccelCalibration
	* @brief Clase para representar celdas.
	*/
	class IMUCalibration
    {
    public:
    /* Public functions */
        IMUCalibration();
        ~IMUCalibration();
        int calibrate(void);
        int correctAccelValues(const Eigen::Ref<const Eigen::MatrixXd> accel_in, Eigen::Ref<Eigen::MatrixXd> accel_out);
        int plotAccel(void);
        void plotData(const Eigen::Ref<const Eigen::MatrixXd> data_values, double data_rate, std::string x_label, std::string y_label, std::string title);
        void plotData(const Eigen::Ref<const Eigen::MatrixXd> data_values, const Eigen::Ref<const Eigen::VectorXd> points_status, double data_rate, std::string x_label, std::string y_label, std::string title);
        int plotGyro(void);
        int start(void);

    private:
        /* Private variables */

        // Node handle
        ros::NodeHandle nh_;


        std::string bag_file_;
        std::string topic_name_;
        double data_rate_;
        double t_init_;
        int tinit_samples_;
        double t_w_;
        int t_w_samples_;
        double e_init_k_;
        double gravity_;
	    Eigen::MatrixXd accel_values_, accel_values_corr_, accel_values_prom_;
        Eigen::MatrixXd gyro_values_, gyro_values_corr_;
        Eigen::MatrixXd mag_values_, mag_values_corr_;

        Eigen::VectorXd static_data_;
        Eigen::MatrixX2i static_timestep_;

        int debug_;

        double e_init_;

        bool start_;

        bool plot_data_;

        AccelCalibration accel_cal_;
        GyroCalibration gyro_cal_;

        /* Private functions */
        int init(void);
        int readBag(void);
        int getStaticDetectorCoefficient(void);
        int getStaticIntervals(void);
        int getVariance(const Eigen::Ref<const Eigen::MatrixXd> values, Eigen::Ref<Eigen::Vector3d> variance);
    };
};

#endif //_IMU_CALIBRATION_H