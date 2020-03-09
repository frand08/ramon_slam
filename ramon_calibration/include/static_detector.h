#include <Eigen/Eigen>

int get_variance(Eigen::MatrixXf accel_values, Eigen::Vector3f &var);
int get_static_detector_coeff(Eigen::MatrixXf accel_init_values, float &static_det_coeff);
