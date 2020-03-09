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

int get_static_detector_coeff(Eigen::MatrixXf accel_init_values, float &static_det_coeff)
{
    Eigen::Vector3f var;
    if(get_variance(accel_init_values, var) < 0)
    {
        return -1;
    }

    static_det_coeff = sqrt(var(0) * var(0) + var(1) * var(1) + var(2) * var(2));

    return 0;
}

int get_static_intervals_prom(Eigen::MatrixXf accel_values, int tw_samples, Eigen::MatrixXf &static_intervals_prom)
{

}
