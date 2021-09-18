/**
 * @file gyro_calibration.h
 * @author Francisco A. Dominguez (dominguezfranciscoa@gmail.com)
 * @brief
 * @version 0.1
 * @date 2020-06-20
 *
 * @copyright Copyright (c) 2020
 *
 */

#ifndef _GYRO_CALIBRATION_H
#define _GYRO_CALIBRATION_H

#include "gyro_functor.h"

namespace ramon_calibration
{
	/**
	* @class GyroCalibration
	* @brief Gyroscope calibration class.
	*/
	class GyroCalibration
	{
		public:
			GyroCalibration();
			GyroCalibration(Eigen::Vector3d noise);
			GyroCalibration(Eigen::Matrix3d T_init, Eigen::Matrix3d K_init);
			GyroCalibration(Eigen::Matrix3d T_init, Eigen::Matrix3d K_init, Eigen::Vector3d noise);
			~GyroCalibration();
			Eigen::LevenbergMarquardtSpace::Status calibrate(const Eigen::Ref<const Eigen::MatrixX3d> accel_values, const Eigen::Ref<const Eigen::MatrixX3d> gyro_values, const Eigen::Ref<const Eigen::MatrixX2i> static_times, int tinit_values, double dt);
			int correctValues(const Eigen::Ref<const Eigen::MatrixXd> gyro_values, Eigen::Ref<Eigen::MatrixXd> gyro_values_corr);
			int getBias(Eigen::Ref<Eigen::Vector3d> b);
			int getParams(Eigen::Ref<Eigen::Matrix3d> T, Eigen::Ref<Eigen::Matrix3d> K, Eigen::Ref<Eigen::Vector3d> b);
			int getT(Eigen::Ref<Eigen::Matrix3d> T);
			int getK(Eigen::Ref<Eigen::Matrix3d> K);
			
			int printCalibratedValues(void);
			void setInitalValues(Eigen::Matrix3d T_init, Eigen::Matrix3d K_init);
			void setNoise(Eigen::Vector3d noise);
			int start(int tinit_values);
		private:
			int n_;
			int m_;
			bool calibrated_;
			
			Eigen::Matrix3d T_;
			double gamma_yz_, gamma_zy_, gamma_zx_, gamma_xz_, gamma_xy_, gamma_yx_;

			Eigen::Matrix3d K_;
			double s_x_, s_y_, s_z_;

			Eigen::Vector3d b_;

			bool use_noise_;
			Eigen::Vector3d noise_;
			double noise_x_, noise_y_, noise_z_;
	};
};

#endif //_GYRO_CALIBRATION_H