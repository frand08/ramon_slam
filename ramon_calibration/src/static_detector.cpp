#include "static_detector.h"

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

int get_static_intervals_prom(Eigen::MatrixXf accel_values, int tw_samples, float e_init, Eigen::VectorXf &points_status, Eigen::MatrixXf &static_intervals_prom)
{
    int count_tw = int(accel_values.rows() / tw_samples);
    Eigen::Vector3f var_aux;
    float e_tw;             // Static detector coeff
    int i = 0;
    
    points_status.resize(accel_values.col(0).rows());
    while(i < accel_values.rows() - count_tw)
    {
        if(get_variance(accel_values.block(i,0,tw_samples,3), var_aux) >= 0)
        {
            e_tw = sqrt(var_aux(0) * var_aux(0) + var_aux(1) * var_aux(1) + var_aux(2) * var_aux(2));
            if(e_tw < 0.3 * e_init)
            {
                points_status.segment(i,tw_samples) = Eigen::VectorXf::Constant(tw_samples, 5);
                i += tw_samples;
            }
            else
            {
                points_status(i) = 0.0;
                i++;
            }
        }
    }
}
