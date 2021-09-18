/**
 * @file accel_calibration.h
 * @author Francisco A. Dominguez (dominguezfranciscoa@gmail.com)
 * @brief
 * @version 0.1
 * @date 2020-06-20
 *
 * @copyright Copyright (c) 2020
 *
 */

#ifndef _ACCEL_CALIBRATION_H
#define _ACCEL_CALIBRATION_H

#include "accel_functor.h"

namespace ramon_calibration
{
	/**
	* @class AccelCalibration
	* @brief Accelerometer calibration class.
	*/
	class AccelCalibration
	{
		public:
			AccelCalibration();
			AccelCalibration(Eigen::Vector3d noise);
			AccelCalibration(Eigen::Matrix3d T_init, Eigen::Matrix3d K_init, Eigen::Vector3d b_init);
			AccelCalibration(Eigen::Matrix3d T_init, Eigen::Matrix3d K_init, Eigen::Vector3d b_init, Eigen::Vector3d noise);
			~AccelCalibration();
			Eigen::LevenbergMarquardtSpace::Status calibrate(const Eigen::Ref<const Eigen::MatrixXd> accel_values);
			int correctValues(const Eigen::Ref<const Eigen::MatrixX3d> accel_values, Eigen::Ref<Eigen::MatrixX3d> accel_values_corr);
			int getBias(Eigen::Ref<Eigen::Vector3d> b);
			int getParams(Eigen::Ref<Eigen::Matrix3d> T, Eigen::Ref<Eigen::Matrix3d> K, Eigen::Ref<Eigen::Vector3d> b);
			int getT(Eigen::Ref<Eigen::Matrix3d> T);
			int getK(Eigen::Ref<Eigen::Matrix3d> K);
			
			int printCalibratedValues(void);
			
			void setInitalValues(Eigen::Matrix3d T_init, Eigen::Matrix3d K_init, Eigen::Vector3d b_init);
			void setNoise(Eigen::Vector3d noise);
		private:
			int n_;
			int m_;
			bool calibrated_;
			
			Eigen::Matrix3d T_;
			double alpha_yz_, alpha_zy_, alpha_zx_;

			Eigen::Matrix3d K_;
			double s_x_, s_y_, s_z_;

			Eigen::Vector3d b_;
			double b_x_, b_y_, b_z_;

			bool use_noise_;
			Eigen::Vector3d noise_;
			double noise_x_, noise_y_, noise_z_;
	};
};

#endif //_ACCEL_CALIBRATION_H