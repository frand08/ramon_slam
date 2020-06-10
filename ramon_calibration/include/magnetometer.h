#include <Eigen/Eigen>
#include <Eigen/Dense>

#include <iostream>
#include <stdio.h>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/KroneckerProduct>

struct mag_cal_functor
{
	// 'm' pairs of (x, f(x))
	Eigen::MatrixXf gyro_values;
	Eigen::MatrixXf mag_values;
	Eigen::Vector4f q_init;
	Eigen::Vector3f b_g;
    float dt_factor, dt;
    float c1, c2, c3, c4;
    float a21, a31, a41, a32, a42, a43;
	Eigen::Vector3f gyro_measnoise;

	// Compute 'm' errors, one for each data point, for the given parameter values in 'x'
	int operator()(const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const
	{
		// 'x' has dimensions n x 1
		// It contains the current estimates for the parameters.

		// 'fvec' has dimensions m x 1
		// It will contain the error for each data point.

        Eigen::Vector4f qk, qk_1, q1, q2, q3, q4, k1, k2, k3, k4;
		Eigen::Quaternionf quat;
        Eigen::Vector3f m_n;
        Eigen::Vector3f mk_b;
        Eigen::Matrix3f C_si;
        Eigen::Matrix3f C_sc;
        Eigen::Matrix3f C_no;
        Eigen::Matrix3f R_bmb;
        Eigen::Vector3f o_hi;
        Eigen::Vector3f o_zb;
        Eigen::Matrix3f D;
        Eigen::Vector3f o;

        Eigen::Vector3f e_mt;

        float dip_angle = x(0);

		float C_sc_11 = x(0);
		float C_sc_12 = x(0);
		float C_sc_13 = x(0);
		float C_sc_21 = x(0);
		float C_sc_22 = x(0);
		float C_sc_23 = x(0);
		float C_sc_31 = x(0);
		float C_sc_32 = x(0);
		float C_sc_33 = x(0);

		C_sc << C_sc_11, C_sc_12, C_sc_13,
				C_sc_21, C_sc_22, C_sc_23,
				C_sc_31, C_sc_23, C_sc_33;

		float C_no_11 = x(0);
		float C_no_12 = x(0);
		float C_no_13 = x(0);
		float C_no_21 = x(0);
		float C_no_22 = x(0);
		float C_no_23 = x(0);
		float C_no_31 = x(0);
		float C_no_32 = x(0);
		float C_no_33 = x(0);

		C_no << C_no_11, C_no_12, C_no_13,
				C_no_21, C_no_22, C_no_23,
				C_no_31, C_no_23, C_no_33;

		float C_si_11 = x(0);
		float C_si_12 = x(0);
		float C_si_13 = x(0);
		float C_si_21 = x(0);
		float C_si_22 = x(0);
		float C_si_23 = x(0);
		float C_si_31 = x(0);
		float C_si_32 = x(0);
		float C_si_33 = x(0);

		C_si << C_si_11, C_si_12, C_si_13,
				C_si_21, C_si_22, C_si_23,
				C_si_31, C_si_23, C_si_33;

		float R_bmb_11 = x(0);
		float R_bmb_12 = x(0);
		float R_bmb_13 = x(0);
		float R_bmb_21 = x(0);
		float R_bmb_22 = x(0);
		float R_bmb_23 = x(0);
		float R_bmb_31 = x(0);
		float R_bmb_32 = x(0);
		float R_bmb_33 = x(0);

		R_bmb << R_bmb_11, R_bmb_12, R_bmb_13,
				 R_bmb_21, R_bmb_22, R_bmb_23,
				 R_bmb_31, R_bmb_23, R_bmb_33;

		float o_hi_1 = x(0);
		float o_hi_2 = x(0);
		float o_hi_3 = x(0);

		o_hi << o_hi_1,
				o_hi_2,
				o_hi_3;
				
		float o_zb_1 = x(0);
		float o_zb_2 = x(0);
		float o_zb_3 = x(0);

		o_zb << o_zb_1,
				o_zb_2,
				o_zb_3;
				
        Eigen::Vector3f xValue;             // w_s

        float yValue;

        Eigen::Vector3f w_o;
        Eigen::Matrix4f omega;

        yValue = 0;

        D = C_sc * C_no * C_si * R_bmb;
        o = C_sc * C_no * o_hi + o_zb;
		// Set gravity versor initial value to [0 0 g]
		m_n << cos(dip_angle),
               0,
		       -sin(dip_angle);

		qk = q_init;
		
		// RK4n needs intermediate values, so we set the 
		// dt = dt_real * 2 --> dt_factor = 2
		// for (int i = 0; i < values(); i+=2) 
		for (int i = 1; i < values(); i+=2)
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

            w_o << gyro_values(int(i + c1 * dt_factor), 0),         // w_x
                   gyro_values(int(i + c1 * dt_factor), 1),         // w_y
                   gyro_values(int(i + c1 * dt_factor), 2);         // w_z

			omega <<   0   , -w_o(0), -w_o(1), -w_o(2),
					 w_o(0),    0   ,  w_o(2), -w_o(1),
					 w_o(1), -w_o(2),    0   , 	w_o(0),
					 w_o(2),  w_o(1), -w_o(0),    0   ;

			//k1
			q1 = qk;					 
			k1 = 0.5 * omega * q1;

			//k2
			q2 = qk + dt * dt_factor * a21 * k1;

            w_o << gyro_values(int(i + c2 * dt_factor), 0),         // w_x
                   gyro_values(int(i + c2 * dt_factor), 1),         // w_y
                   gyro_values(int(i + c2 * dt_factor), 2);         // w_z

			omega <<   0   , -w_o(0), -w_o(1), -w_o(2),
					 w_o(0),    0   ,  w_o(2), -w_o(1),
					 w_o(1), -w_o(2),    0   , 	w_o(0),
					 w_o(2),  w_o(1), -w_o(0),    0   ;
			k2 = 0.5 * omega * q2;

			// k3
			q3 = qk + dt * dt_factor * (a31 * k1 + a32 * k2);

            w_o << gyro_values(int(i + c3 * dt_factor), 0),         // w_x
                   gyro_values(int(i + c3 * dt_factor), 1),         // w_y
                   gyro_values(int(i + c3 * dt_factor), 2);         // w_z

			omega <<   0   , -w_o(0), -w_o(1), -w_o(2),
					 w_o(0),    0   ,  w_o(2), -w_o(1),
					 w_o(1), -w_o(2),    0   , 	w_o(0),
					 w_o(2),  w_o(1), -w_o(0),    0   ;
			k3 = 0.5 * omega * q3;

			// k4
			q4 = qk + dt * dt_factor * (a41 * k1 + a42 * k2 + a43 * k3);

            w_o << gyro_values(int(i + c4 * dt_factor), 0),         // w_x
                   gyro_values(int(i + c4 * dt_factor), 1),         // w_y
                   gyro_values(int(i + c4 * dt_factor), 2);         // w_z

			omega <<   0   , -w_o(0), -w_o(1), -w_o(2),
					 w_o(0),    0   ,  w_o(2), -w_o(1),
					 w_o(1), -w_o(2),    0   , 	w_o(0),
					 w_o(2),  w_o(1), -w_o(0),    0   ;
			k4 = 0.5 * omega * q4;			

            qk_1 = qk + dt * dt_factor * (k1 / 6 + k2 / 3 + k3 / 3 + k4 / 6);

            qk_1 = qk_1 / qk_1.norm();

            qk = qk_1;

			// Apply 3D rotation to the normalized local magnetic field m_n (n -> navigation frame)
			quat.w() = qk_1(0);
			quat.x() = qk_1(1);
			quat.y() = qk_1(2);
			quat.z() = qk_1(3);

			mk_b = D * quat.normalized().toRotationMatrix() * m_n + o + e_mt;

            fvec(i) = yValue - (mag_values.row(i).transpose() - mk_b).squaredNorm();

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








struct mag_cal_init_firststep_functor
{
	// 'm' pairs of (x, f(x))
	Eigen::MatrixXf mag_values;

	// Compute 'm' errors, one for each data point, for the given parameter values in 'x'
	int operator()(const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const
	{
		// 'x' has dimensions n x 1
		// It contains the current estimates for the parameters.

		// 'fvec' has dimensions m x 1
		// It will contain the error for each data point.

		Eigen::Matrix3f A;
		float a_11, a_12, a_13, a_21, a_22, a_23, a_31, a_32, a_33;
		Eigen::Vector3f b;
		float b_1, b_2, b_3;
		float c;
		
		a_11 = x(0);
		a_12 = x(1);
		a_21 = x(2);
		a_22 = x(3);
		b_1 = x(4);
		b_2 = x(5);
		b_3 = x(6);
		c = x(7);

		A << a_11, a_12, a_13,
			 a_21, a_22, a_23,
			 a_31, a_32, a_33;

        Eigen::Vector3f xValue;

        float yValue;
		float aux1, aux2;

        yValue = 0;

		fvec(0) = 1 - A.trace(); 	// To avoid the 0 solution
		for (int i = 1; i < values(); i++)
        {
			xValue << mag_values(i, 0),     // m_x
					  mag_values(i, 1),     // m_y
					  mag_values(i, 2);     // m_z

			aux1 = xValue.transpose()*A*xValue;
			aux2 = b.transpose()*xValue;
            fvec(i) = aux1 + aux2 + c;
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