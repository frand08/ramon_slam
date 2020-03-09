#include <Eigen/Eigen>
#include <Eigen/Dense>

#include <iostream>
#include <stdio.h>
#include <unsupported/Eigen/NonLinearOptimization>

struct gyro_cal_functor
{
	// 'm' pairs of (x, f(x))
	Eigen::MatrixXf gyro_values;
	Eigen::MatrixXf accel_values;
	Eigen::Vector4f q_init;
	Eigen::Vector3f b_g;
    float dt_factor, dt;
    float c1, c2, c3, c4;
    float a21, a31, a41, a32, a42, a43;

	// Compute 'm' errors, one for each data point, for the given parameter values in 'x'
	int operator()(const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const
	{
		// 'x' has dimensions n x 1
		// It contains the current estimates for the parameters.

		// 'fvec' has dimensions m x 1
		// It will contain the error for each data point.

        Eigen::Vector4f qk, qk_1, q1, q2, q3, q4, k1, k2, k3, k4;
		Eigen::Quaternionf quat;
		Eigen::Vector3f gravity_versor, ug_k_1;
        Eigen::Matrix3f T_g;
        Eigen::Matrix3f K_g;
        // Eigen::Vector3f b_g;

        float g_yz = x(0);
        float g_zy = x(1);
        float g_zx = x(2);
        float g_xz = x(3);
        float g_xy = x(4);
        float g_yx = x(5);        
        float sg_x = x(6);
        float sg_y = x(7);
        float sg_z = x(8);

        Eigen::Vector3f xValue;             // w_s

        float yValue;                       // ||g||^2

        Eigen::Vector3f w_o;
        Eigen::Matrix4f omega;

        T_g <<   1  , -g_yz,  g_zy,
                g_xz,   1  , -g_zx,
               -g_xy,  g_yx,   1  ;  

        K_g << sg_x,  0  ,   0  ,
                0  , sg_y,   0  ,
                0  ,  0  ,  sg_z;

        yValue = 0;

		gravity_versor << 0,
						  0,
						  1;

		qk = q_init;
		
		// RK4n needs intermediate values, so we set the 
		// dt = dt_real * 2 --> dt_factor = 2
		// for (int i = 0; i < values(); i+=2) 
		for (int i = 0; i < values(); i+=2)
        {
			if(i+dt_factor >= values())
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

			//k1
			q1 = qk;

            xValue << gyro_values(int(i + c1 * dt_factor), 0),         // w_x
                      gyro_values(int(i + c1 * dt_factor), 1),         // w_y
                      gyro_values(int(i + c1 * dt_factor), 2);         // w_z
			w_o = T_g * K_g * (xValue + b_g);
			omega <<   0   , -w_o(0), -w_o(1), -w_o(2),
					 w_o(0),    0   ,  w_o(2), -w_o(1),
					 w_o(1), -w_o(2),    0   , 	w_o(0),
					 w_o(2),  w_o(1), -w_o(0),    0   ;
			k1 = 0.5 * omega * q1;

			//k2
			q2 = qk + dt * dt_factor * a21 * k1;

            xValue << gyro_values(int(i + c2 * dt_factor), 0),         // w_x
                      gyro_values(int(i + c2 * dt_factor), 1),         // w_y
                      gyro_values(int(i + c2 * dt_factor), 2);         // w_z
			w_o = T_g * K_g * (xValue + b_g);
			omega <<   0   , -w_o(0), -w_o(1), -w_o(2),
					 w_o(0),    0   ,  w_o(2), -w_o(1),
					 w_o(1), -w_o(2),    0   , 	w_o(0),
					 w_o(2),  w_o(1), -w_o(0),    0   ;
			k2 = 0.5 * omega * q2;

			// k3
			q3 = qk + dt * dt_factor * (a31 * k1 + a32 * k2);

            xValue << gyro_values(int(i + c3 * dt_factor), 0),         // w_x
                      gyro_values(int(i + c3 * dt_factor), 1),         // w_y
                      gyro_values(int(i + c3 * dt_factor), 2);         // w_z
			w_o = T_g * K_g * (xValue + b_g);
			omega <<   0   , -w_o(0), -w_o(1), -w_o(2),
					 w_o(0),    0   ,  w_o(2), -w_o(1),
					 w_o(1), -w_o(2),    0   , 	w_o(0),
					 w_o(2),  w_o(1), -w_o(0),    0   ;
			k3 = 0.5 * omega * q3;

			// k4
			q4 = qk + dt * dt_factor * (a41 * k1 + a42 * k2 + a43 * k3);

            xValue << gyro_values(int(i + c4 * dt_factor), 0),         // w_x
                      gyro_values(int(i + c4 * dt_factor), 1),         // w_y
                      gyro_values(int(i + c4 * dt_factor), 2);         // w_z
			w_o = T_g * K_g * (xValue + b_g);
			omega <<   0   , -w_o(0), -w_o(1), -w_o(2),
					 w_o(0),    0   ,  w_o(2), -w_o(1),
					 w_o(1), -w_o(2),    0   , 	w_o(0),
					 w_o(2),  w_o(1), -w_o(0),    0   ;
			k4 = 0.5 * omega * q4;			

            qk_1 = qk + dt * dt_factor * (k1 / 6 + k2 / 3 + k3 / 3 + k4 / 6);

			// Apply 3D rotation to gravity versor (0 0 1)
			quat.w() = qk_1(0);
			quat.x() = qk_1(1);
			quat.y() = qk_1(2);
			quat.z() = qk_1(3);

			// cost fnc -> ||ua_k - ug_k||^2 = yValue = 0
			ug_k_1 = quat.normalized().toRotationMatrix() * gravity_versor;
            // fvec(i) = yValue - (accel_values.row(i+dt_factor).transpose() - ug_k_1).squaredNorm();
            fvec(i) = (accel_values.row(i+dt_factor).transpose() - ug_k_1).squaredNorm();

            qk_1 = qk_1 / qk_1.norm();

            qk = qk_1;
		}
		return 0;
	}

	// Compute the jacobian of the errors
	int df(const Eigen::VectorXf &x, Eigen::MatrixXf &fjac) const
	{
		// 'x' has dimensions n x 1
		// It contains the current estimates for the parameters.

		// 'fjac' has dimensions m x n
		// It will contain the jacobian of the errors, calculated numerically in this case.

		float epsilon;
		epsilon = 1e-5f;

		for (int i = 0; i < x.size(); i++) {
			Eigen::VectorXf xPlus(x);
			xPlus(i) += epsilon;
			Eigen::VectorXf xMinus(x);
			xMinus(i) -= epsilon;

			Eigen::VectorXf fvecPlus(values());
			operator()(xPlus, fvecPlus);

			Eigen::VectorXf fvecMinus(values());
			operator()(xMinus, fvecMinus);

			Eigen::VectorXf fvecDiff(values());
			fvecDiff = (fvecPlus - fvecMinus) / (2.0f * epsilon);

			fjac.block(0, i, values(), 1) = fvecDiff;
		}

		return 0;
	}

	// Number of data points, i.e. values.
	int m;

	// Returns 'm', the number of values.
	int values() const { return m; }

	// The number of parameters, i.e. inputs.
	int n;

	// Returns 'n', the number of inputs.
	int inputs() const { return n; }

};

int gyro_calibration(Eigen::MatrixXf accel_values, Eigen::MatrixXf gyro_values, float t_init, Eigen::VectorXf &output_values);
