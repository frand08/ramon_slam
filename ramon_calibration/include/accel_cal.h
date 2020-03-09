#include <Eigen/Eigen>

#include <unsupported/Eigen/NonLinearOptimization>

struct accel_cal_functor
{
	// 'm' pairs of (x, f(x))
	Eigen::MatrixXf accel_values;

	// Compute 'm' errors, one for each data point, for the given parameter values in 'x'
	int operator()(const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const
	{
		// 'x' has dimensions n x 1
		// It contains the current estimates for the parameters.

		// 'fvec' has dimensions m x 1
		// It will contain the error for each data point.

        Eigen::Matrix3f T_a;
        Eigen::Matrix3f K_a;
        Eigen::Vector3f b_a;

        float a_yz = x(0);
        float a_zy = x(1);
        float a_zx = x(2);
        float sa_x = x(3);
        float sa_y = x(4);
        float sa_z = x(5);
        float ba_x = x(6);
        float ba_y = x(7);
        float ba_z = x(8);

        T_a << 1, -a_yz, a_zy ,
               0,   1  , -a_zx,
               0,   0  ,   1  ;  

        K_a << sa_x,  0  ,   0  ,
                0  , sa_y,   0  ,
                0  ,  0  ,  sa_z;
        
        b_a << ba_x,
                ba_y,
                ba_z;         

		for (int i = 0; i < values(); i++) 
        {
            Eigen::Vector3f xValue;              	// a_s
            xValue << accel_values(i, 0),         	// a_x
                      accel_values(i, 1),         	// a_y
                      accel_values(i, 2);         	// a_z

            float yValue = 0;                       // ||g||^2

            Eigen::Vector3f a_o = T_a * K_a * (xValue + b_a);
            fvec(i) = yValue - pow(1 - a_o.squaredNorm(),2);
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

int accel_calibration(Eigen::MatrixXf accel_values, Eigen::VectorXf &output_values);