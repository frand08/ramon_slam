#include "static_detector.h"
#include <iostream>

int get_variance(Eigen::MatrixXf matrix, Eigen::Vector3f &var)
{
    Eigen::MatrixXf mean_values, mean_matrix(matrix.rows(),matrix.cols());

    if(matrix.rows() <= 1)
    {
        return -1;
    }
    mean_values = matrix.colwise().mean();

    mean_matrix << Eigen::MatrixXf::Constant(matrix.rows(),1,mean_values(0)), 
                   Eigen::MatrixXf::Constant(matrix.rows(),1,mean_values(1)), 
                   Eigen::MatrixXf::Constant(matrix.rows(),1,mean_values(2));
    var = ((matrix - mean_matrix).array() * (matrix - mean_matrix).array()).colwise().sum().transpose();
    var /= (matrix.rows() - 1);

    return 0;
}

int get_static_detector_coeff(Eigen::MatrixXf values, float &static_det_coeff)
{
    Eigen::Vector3f var;
    if(get_variance(values, var) < 0)
    {
        return -1;
    }
    static_det_coeff = sqrt(var(0) * var(0) + var(1) * var(1) + var(2) * var(2));

    return 0;
}

int get_static_intervals_prom(Eigen::MatrixXf accel_values, int tw_samples, int tinit_samples, float e_init, float e_init_k, Eigen::VectorXf &points_status, Eigen::MatrixXf &static_intervals_prom)
{
    int count_tw = int(accel_values.rows() / tw_samples);
    Eigen::Vector3f var_aux;
    float e_tw;             // Static detector coeff
    int i = tinit_samples;
    bool static_interval = false;
    int index = 0;
    int j = 0;

    static_intervals_prom.resize(1,3);
    points_status.resize(accel_values.col(0).rows());
    while(i < accel_values.rows() - tw_samples)
    {
        if(get_variance(accel_values.block(i,0,tw_samples,3), var_aux) >= 0)
        {
            e_tw = sqrt(var_aux(0) * var_aux(0) + var_aux(1) * var_aux(1) + var_aux(2) * var_aux(2));
            if(e_tw < 2.0 * e_init)
            {
                static_interval = true;
                index = i+1;
                points_status.segment(i,tw_samples) = Eigen::VectorXf::Constant(tw_samples, 2);
                i += tw_samples;
            }
            else
            {
                if(static_interval)
                {
                    static_interval = false;
                    /* TODO: Check if correct */
                    static_intervals_prom.row(j) = accel_values.block(index, 0, i - index, 3).colwise().mean();
                    j++;
                    static_intervals_prom.conservativeResize(j+1,3);
                }
                points_status(i) = 0.0;
                i++;
            }
        }
    }
    static_intervals_prom.conservativeResize(j,3);
    return 0;
}
