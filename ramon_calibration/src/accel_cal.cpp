#include "accel_cal.h"

int accel_calibration(Eigen::MatrixXf accel_values, Eigen::VectorXf &output_values)
{
    float a_yz, a_zy, a_zx;
    float sa_x, sa_y, sa_z;
    float ba_x, ba_y, ba_z;
    float as_x, as_y, as_z;

    int n = 9;              // Number of variables
    int m;                  // Number of x values

    Eigen::VectorXf x(n);   // Values to get by LM algorithm


    // axis missalignments (init values)
    a_yz = 0.0;
    a_zy = 0.0;
    a_zx = 0.0;

    // axis scales (init values)
    sa_x = 1.0;
    sa_y = 1.0;
    sa_z = 1.0;

    // biases (init values)

    ba_x = 0.0;
    ba_y = 0.0;
    ba_z = 0.0;
    
    // Assign the initial values
    x(0) = a_yz;
    x(1) = a_zy;
    x(2) = a_zx;
    x(3) = sa_x;
    x(4) = sa_y;
    x(5) = sa_z;
    x(6) = ba_x;
    x(7) = ba_y;
    x(8) = ba_z;

    int j = 0;
    m = accel_values.rows();

    accel_cal_functor functor;
    functor.accel_values = accel_values;
    functor.m = m;
    functor.n = n;

    Eigen::LevenbergMarquardt<accel_cal_functor, float> lm(functor);
    int status = lm.minimize(x);

    output_values << x(0),
                     x(1),
                     x(2),
                     x(3),
                     x(4),
                     x(5),
                     x(6),
                     x(7),
                     x(8);
                     
    return 0;
}