/**
 * @file accel_functor.h
 * @author Francisco A. Dominguez (dominguezfranciscoa@gmail.com)
 * @brief
 * @version 0.1
 * @date 2020-06-20
 *
 * @copyright Copyright (c) 2020
 *
 */

#ifndef _ACCEL_FUNCTOR_H
#define _ACCEL_FUNCTOR_H

#include "generic_functor.h"
#include <iostream>

struct accel_cal_functor: Functor<float>
{
	Eigen::MatrixXf accel_values;

	accel_cal_functor(const Eigen::Ref<const Eigen::MatrixXf> accel, 
					 int val, int in): 
					 accel_values(accel),
					 Functor<float>(in, val) {}

	// Compute 'm' errors, one for each data point, for the given parameter values in 'x'
	int operator()(const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const
	{
		// 'x' has dimensions n x 1
		// It contains the current estimates for the parameters.

		// 'fvec' has dimensions m x 1
		// It will contain the error for each data point.

		static int iterations = 1;
        Eigen::Matrix3f T;
        Eigen::Matrix3f K;
        Eigen::Vector3f b;

		Eigen::Vector3f a_o;

		Eigen::Vector3f xValue;              	// a_s
		float yValue = 0;                       // ||g||^2		

        float a_yz = x(0);
        float a_zy = x(1);
        float a_zx = x(2);
        float s_x = x(3);
        float s_y = x(4);
        float s_z = x(5);
        float b_x = x(6);
        float b_y = x(7);
        float b_z = x(8);

        T << 1, -a_yz,  a_zy,
             0,   1  , -a_zx,
             0,   0  ,   1  ;  

        K << s_x ,  0  ,  0  ,
              0  , s_y ,  0  ,
              0  ,  0  , s_z ;
        
        b << b_x,
             b_y,
             b_z;         
		
		std::cout << "Number of iterations: " << iterations << std::endl;
		if(iterations == 1)
		{
			std::cout << "values: " << values() << std::endl;
			std::cout << "inputs: " << inputs() << std::endl;
		}
		iterations++;

		for (int i = 0; i < values(); i++) 
        {
            xValue << accel_values(i, 0),         	// a_x
                      accel_values(i, 1),         	// a_y
                      accel_values(i, 2);         	// a_z


            Eigen::Vector3f a_o = T * K * (xValue + b);

			/* FIXME: CREO QUE ESTA MAL CALCULADO!! NO DEBERIA PONER SOLO LO QUE NO VA COMO NORMA? */
            fvec(i) = 1 - a_o.squaredNorm();
		}
		return 0;
	}
};

#endif //ACCEL_FUNCTOR_H