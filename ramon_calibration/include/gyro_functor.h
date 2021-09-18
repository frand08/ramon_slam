/**
 * @file gyro_functor.h
 * @author Francisco A. Dominguez (dominguezfranciscoa@gmail.com)
 * @brief
 * @version 0.1
 * @date 2020-06-20
 *
 * @copyright Copyright (c) 2020
 *
 */

#ifndef _GYRO_FUNCTOR_H
#define _GYRO_FUNCTOR_H

#include "generic_functor.h"
#include <iostream>


struct gyro_cal_functor: Functor<double>
{
	Eigen::MatrixX3d gyro_values;
	Eigen::MatrixX3d accel_values;
	Eigen::Vector4d q_init;
	Eigen::Vector3d bias;
    double dt_factor, dt;
	Eigen::Vector3d measnoise;
	Eigen::MatrixX2i static_intervals;

	gyro_cal_functor(const Eigen::Ref<const Eigen::MatrixX3d> accel, 
					 const Eigen::Ref<const Eigen::MatrixX3d> gyro,
					 const Eigen::Ref<const Eigen::MatrixX2i> intervals, 
					 double dt, double dt_factor, int val, int in,
					 Eigen::Vector3d b, Eigen::Vector3d noise,
					 Eigen::Vector4d q): 
					 accel_values(accel),
					 gyro_values(gyro),
					 static_intervals(intervals),
					 dt(dt),
					 dt_factor(dt_factor),
					 bias(b), measnoise(noise),
					 q_init(q),
					//  Functor<double>(in, val*3) {}
					 Functor<double>(in, intervals.rows()) {}

	// Compute 'm' errors, one for each data point, for the given parameter values in 'x'
	int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
	{
		// 'x' has dimensions n x 1
		// It contains the current estimates for the parameters.

		// 'fvec' has dimensions m x 1
		// It will contain the error for each data point.
		static int iterations = 1;
        Eigen::Vector4d qk_reg, qk_next, q1, q2, q3, q4, k1, k2, k3, k4;
		Eigen::Quaterniond quat;
		Eigen::Vector3d gravity_versor, ug_k_1;
        Eigen::Matrix3d T_g;
        Eigen::Matrix3d K_g;
		
		double c1, c2, c3, c4;
		double a21, a31, a41, a32, a42, a43;

        double g_yz = x(0);
        double g_zy = x(1);
        double g_zx = x(2);
        double g_xz = x(3);
        double g_xy = x(4);
        double g_yx = x(5);        
        double sg_x = x(6);
        double sg_y = x(7);
        double sg_z = x(8);

        Eigen::Vector3d xValue;             // w_s

		Eigen::Vector3d aux, aux2;

        Eigen::Vector3d w_o;
        Eigen::Matrix4d omega;

        T_g <<   1  , -g_yz,  g_zy,
                g_xz,   1  , -g_zx,
               -g_xy,  g_yx,   1  ;  

        K_g << sg_x,  0  ,   0  ,
                0  , sg_y,   0  ,
                0  ,  0  ,  sg_z;

		// Set gravity versor initial value to [0 0 g]
		gravity_versor << 0,
						  0,
						  1;

		// Load params respect to Runge-Kutta 4th order method
		c1 = 0;
		c2 = 0.5;
		c3 = 0.5;
		c4 = 1;

		a21 = 0.5;
		a31 = 0;
		a41 = 0;
		a32 = 0.5;
		a42 = 0;
		a43 = 1;

		int f_index = 0;
		
		std::cout << "Number of iterations: " << iterations << std::endl;
		if(iterations == 1)
		{
			std::cout << "values: " << values() << std::endl;
			std::cout << "inputs: " << inputs() << std::endl;
		}
		iterations++;
		
		// RK4n needs intermediate values, so we set the 
		// dt = dt_real * 2 --> dt_factor = 2

		for (int j = 0; j < static_intervals.rows(); j++)
        {
            // Init q
    		qk_reg = q_init;
			// Get gravity from previous accel value
			gravity_versor = accel_values.row(static_intervals(j,0) - 1).transpose();
			// Integrate over each time stamp
			for(int i = static_intervals(j,0); i < static_intervals(j,1); i++)
			{
				if(i+dt_factor >= static_intervals(j,1))
					break;

				// Geometric Integration of quaternion using RK4n
				
				// qk+1 = qk + dt * (k1 / 6 + k2 / 3 + k3 / 3 + k4 / 6)

				// ki = f(qi,tk + ci * dt)

				//         .
				//f(q,t) = q = 0.5 * omega(w(t)) * q

				//				  i-1
				// qi = qk + dt * sum(aij * kj)
				//				  j=1

				// omega(w) <<  0 , -wx, -wy, -wz,
				//              wx,  0 ,  wz, -wy,
				//              wy, -wz,  0 ,  wx,
				//              wz,  wy, -wx,  0 ;

				// For each step, normalize the (k+1)-th quaternion:
				// qk+1_norm = qk+1 / ||qk+1||

				xValue << gyro_values(int(i + c1 * dt_factor), 0),         // w_x
						  gyro_values(int(i + c1 * dt_factor), 1),         // w_y
						  gyro_values(int(i + c1 * dt_factor), 2);         // w_z
				w_o = T_g * K_g * (xValue + bias + measnoise);
				omega <<   0   , -w_o(0), -w_o(1), -w_o(2),
						 w_o(0),    0   ,  w_o(2), -w_o(1),
						 w_o(1), -w_o(2),    0   , 	w_o(0),
						 w_o(2),  w_o(1), -w_o(0),    0   ;

				//k1
				q1 = qk_reg;					 
				k1 = 0.5 * omega * q1;

				//k2
				q2 = qk_reg + dt * dt_factor * a21 * k1;

				xValue << gyro_values(int(i + c2 * dt_factor), 0),         // w_x
						  gyro_values(int(i + c2 * dt_factor), 1),         // w_y
						  gyro_values(int(i + c2 * dt_factor), 2);         // w_z
				w_o = T_g * K_g * (xValue + bias + measnoise);
				omega <<   0   , -w_o(0), -w_o(1), -w_o(2),
						 w_o(0),    0   ,  w_o(2), -w_o(1),
						 w_o(1), -w_o(2),    0   , 	w_o(0),
						 w_o(2),  w_o(1), -w_o(0),    0   ;
				k2 = 0.5 * omega * q2;

				// k3
				q3 = qk_reg + dt * dt_factor * (a31 * k1 + a32 * k2);

				xValue << gyro_values(int(i + c3 * dt_factor), 0),         // w_x
						  gyro_values(int(i + c3 * dt_factor), 1),         // w_y
						  gyro_values(int(i + c3 * dt_factor), 2);         // w_z
				w_o = T_g * K_g * (xValue + bias + measnoise);
				omega <<   0   , -w_o(0), -w_o(1), -w_o(2),
						 w_o(0),    0   ,  w_o(2), -w_o(1),
						 w_o(1), -w_o(2),    0   , 	w_o(0),
						 w_o(2),  w_o(1), -w_o(0),    0   ;
				k3 = 0.5 * omega * q3;

				// k4
				q4 = qk_reg + dt * dt_factor * (a41 * k1 + a42 * k2 + a43 * k3);

				xValue << gyro_values(int(i + c4 * dt_factor), 0),         // w_x
						  gyro_values(int(i + c4 * dt_factor), 1),         // w_y
						  gyro_values(int(i + c4 * dt_factor), 2);         // w_z
				w_o = T_g * K_g * (xValue + bias + measnoise);
				omega <<   0   , -w_o(0), -w_o(1), -w_o(2),
						 w_o(0),    0   ,  w_o(2), -w_o(1),
						 w_o(1), -w_o(2),    0   , 	w_o(0),
						 w_o(2),  w_o(1), -w_o(0),    0   ;
				k4 = 0.5 * omega * q4;			

				qk_next = qk_reg + dt * dt_factor * (k1 / 6 + k2 / 3 + k3 / 3 + k4 / 6);

                qk_next.normalize();

				qk_reg = qk_next;
			}

			// Apply 3D rotation to gravity versor
 			quat.w() = qk_next(0);
			quat.x() = qk_next(1);
			quat.y() = qk_next(2);
			quat.z() = qk_next(3);
			
			ug_k_1 = quat.toRotationMatrix() * gravity_versor;

            // Block of size (p,q), starting at (i,j) => matrix.block(i,j,p,q);            
			aux = accel_values.block(static_intervals(j,0), 0, static_intervals(j,1) - static_intervals(j,0), 3).colwise().mean().transpose() - ug_k_1;

            fvec(j) = aux.norm();
		}

		return 0;
	}
};
#endif //_GYRO_FUNCTOR_H