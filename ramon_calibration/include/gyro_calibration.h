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
			GyroCalibration(Eigen::Vector3f noise);
			GyroCalibration(Eigen::Matrix3f T_init, Eigen::Matrix3f K_init);
			GyroCalibration(Eigen::Matrix3f T_init, Eigen::Matrix3f K_init, Eigen::Vector3f noise);
			~GyroCalibration();
			Eigen::LevenbergMarquardtSpace::Status calibrate(const Eigen::Ref<const Eigen::MatrixX3f> accel_values, const Eigen::Ref<const Eigen::MatrixX3f> gyro_values, const Eigen::Ref<const Eigen::MatrixX2i> static_times, int tinit_values, float dt);
			int correctValues(const Eigen::Ref<const Eigen::MatrixXf> gyro_values, Eigen::Ref<Eigen::MatrixXf> gyro_values_corr);
			int getBias(Eigen::Ref<Eigen::Vector3f> b);
			int getParams(Eigen::Ref<Eigen::Matrix3f> T, Eigen::Ref<Eigen::Matrix3f> K, Eigen::Ref<Eigen::Vector3f> b);
			int getT(Eigen::Ref<Eigen::Matrix3f> T);
			int getK(Eigen::Ref<Eigen::Matrix3f> K);
			
			int printCalibratedValues(void);
			void setInitalValues(Eigen::Matrix3f T_init, Eigen::Matrix3f K_init);
			void setNoise(Eigen::Vector3f noise);
			int start(int tinit_values);
		private:
			int n_;
			int m_;
			bool calibrated_;
			
			Eigen::Matrix3f T_;
			float gamma_yz_, gamma_zy_, gamma_zx_, gamma_xz_, gamma_xy_, gamma_yx_;

			Eigen::Matrix3f K_;
			float s_x_, s_y_, s_z_;

			Eigen::Vector3f b_;

			bool use_noise_;
			Eigen::Vector3f noise_;
			float noise_x_, noise_y_, noise_z_;
	};
};

#endif //_GYRO_CALIBRATION_H