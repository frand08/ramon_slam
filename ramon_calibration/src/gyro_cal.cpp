#include "gyro_cal.h"

int gyro_calibration(Eigen::MatrixXf accel_values, Eigen::MatrixXf gyro_values, float t_init, Eigen::VectorXf &output_values)
{
    float g_yz, g_zy, g_zx;
    float g_xz, g_xy, g_yx;
    float sg_x, sg_y, sg_z;
    float gs_x, gs_y, gs_z;
    Eigen::Vector3f b_g;

    int n = 9;              // Number of variables
    int m;

    gyro_cal_functor functor;
    Eigen::VectorXf x(n);   // Values to get by LM algorithm

    float freq = 200;
    int bias_counter = int(t_init * freq);

    // Get bias from t_init
    b_g.setZero();
    for(int i = 0; i < bias_counter; i++)
    {
        b_g += gyro_values.row(i).transpose();
    }
    b_g /= bias_counter;

    // axis missalignments (init values)
    g_yz = 0.0;
    g_zy = 0.0;
    g_zx = 0.0;
    g_xz = 0.0;
    g_xy = 0.0;
    g_yx = 0.0;
    
    // axis scales (init values)
    sg_x = 1.0;
    sg_y = 1.0;
    sg_z = 1.0;

    // Assign the initial values
    x(0) = g_yz;
    x(1) = g_zy;
    x(2) = g_zx;
    x(3) = g_xz;
    x(4) = g_xy;
    x(5) = g_yx;
    x(6) = sg_x;
    x(7) = sg_y;
    x(8) = sg_z;

    // Cant Data
    m = gyro_values.rows();     // Number of x values

    functor.accel_values = accel_values;
    functor.gyro_values = gyro_values;

    functor.m = m;
    functor.n = n;

    functor.c1 = 0;
    functor.c2 = 0.5;
    functor.c3 = 0.5;
    functor.c4 = 1;

    functor.a21 = 0.5;
    functor.a31 = 0;
    functor.a41 = 0;
    functor.a32 = 0.5;
    functor.a42 = 0;
    functor.a43 = 1;

    functor.q_init << 1,
                      0,
                      0,
                      0;

    functor.dt_factor = 2;
    functor.dt = 1 / 200;

    functor.b_g = b_g;

    Eigen::LevenbergMarquardt<gyro_cal_functor, float> lm(functor);
    int status = lm.minimize(x);

    output_values << x,
                     b_g;
                     
    // Aca ya deberia tener todos los datos, en los x...
    return 0;
}