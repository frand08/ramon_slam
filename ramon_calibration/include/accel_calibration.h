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
			AccelCalibration(Eigen::Vector3f noise);
			AccelCalibration(Eigen::Matrix3f T_init, Eigen::Matrix3f K_init, Eigen::Vector3f b_init);
			AccelCalibration(Eigen::Matrix3f T_init, Eigen::Matrix3f K_init, Eigen::Vector3f b_init, Eigen::Vector3f noise);
			~AccelCalibration();
			Eigen::LevenbergMarquardtSpace::Status calibrate(const Eigen::Ref<const Eigen::MatrixXf> accel_values);
			int correctValues(const Eigen::Ref<const Eigen::MatrixX3f> accel_values, Eigen::Ref<Eigen::MatrixX3f> accel_values_corr);
			int getBias(Eigen::Ref<Eigen::Vector3f> b);
			int getParams(Eigen::Ref<Eigen::Matrix3f> T, Eigen::Ref<Eigen::Matrix3f> K, Eigen::Ref<Eigen::Vector3f> b);
			int getT(Eigen::Ref<Eigen::Matrix3f> T);
			int getK(Eigen::Ref<Eigen::Matrix3f> K);
			
			int printCalibratedValues(void);
			
			void setInitalValues(Eigen::Matrix3f T_init, Eigen::Matrix3f K_init, Eigen::Vector3f b_init);
			void setNoise(Eigen::Vector3f noise);
		private:
			int n_;
			int m_;
			bool calibrated_;
			
			Eigen::Matrix3f T_;
			float alpha_yz_, alpha_zy_, alpha_zx_;

			Eigen::Matrix3f K_;
			float s_x_, s_y_, s_z_;

			Eigen::Vector3f b_;
			float b_x_, b_y_, b_z_;

			bool use_noise_;
			Eigen::Vector3f noise_;
			float noise_x_, noise_y_, noise_z_;
	};
};

#endif //_ACCEL_CALIBRATION_H