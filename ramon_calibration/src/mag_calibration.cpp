/**
 * @file mag_calibration.cpp
 * @author Francisco A. Dominguez (dominguezfranciscoa@gmail.com)
 * @brief
 * @version 0.1
 * @date 2020-06-20
 *
 * @copyright Copyright (c) 2020
 *
 */

#include "mag_calibration.h"

int mag_calibration(Eigen::MatrixXd accel_values, Eigen::MatrixXd gyro_values, Eigen::MatrixXd mag_values, Eigen::VectorXd &output_values)
{
    mag_cal_misalignment_functor misalignment_functor;
    int n_misalignment = 10;        // Number of variables in the misalignment init
    int m_misalignment;
    Eigen::VectorXd x_misalignment(n_misalignment);

    mag_cal_functor functor;
    int n = 9;                      // Number of variables
    int m;
    Eigen::VectorXd x(n);           // Values to get by LM algorithm

    Eigen::MatrixXd M;
    Eigen::VectorXd zeros;
    Eigen::VectorXd epsilon;
    Eigen::Matrix3d A, aux;
    double a_11, a_12, a_13, a_21, a_22, a_23, a_31, a_32, a_33;
    Eigen::Vector3d b;
    double b_1, b_2, b_3;
    double c;

    double alpha;
    Eigen::Matrix3d Dh_0, Ds_0;
    Eigen::Vector3d oh_0;

    // a_11 = 0.33;
    // a_12 = 0.01;
    // a_13 = 0.01;
    // a_21 = 0.01;
    // a_22 = 0.33;
    // a_23 = 0.01;
    // a_31 = 0.01;
    // a_32 = 0.01;
    // a_33 = 0.34;
    // b_1 = 0.01;
    // b_2 = 0.01;
    // b_3 = 0.01;
    // c = 0.01;

    x_misalignment(0) = a_11;
    x_misalignment(1) = a_12;
    x_misalignment(2) = a_13;
    x_misalignment(3) = a_22;
    x_misalignment(4) = a_23;
    x_misalignment(5) = a_33;
    x_misalignment(6) = b_1;
    x_misalignment(7) = b_2;
    x_misalignment(8) = b_3;
    x_misalignment(9) = c;


    /* TODO: get init values (LM) */
    // M.resize(mag_values.rows(),10);
    // zeros.resize(mag_values.rows(),1);

    // // Condition tr(A) = 1
    // M.row(0) << 1, 0, 0, 0, 1, 0, 0, 0, 1, 0;
    // zeros(0) = 1.0;

    // for(int i = 1; i < mag_values.rows(); i++)
    // {
    //     M.row(i) << Eigen::kroneckerProduct(mag_values.row(i), mag_values.row(i)).eval(), 1;
    //     zeros(i) = 0.0;
    // }
    // epsilon = M.colPivHouseholderQr().solve(zeros);
    // ROS_INFO("Mag misalignment data values");
    // std::cout << "epsilon =\n" << epsilon << std::endl;
    m_misalignment = mag_values.rows();
    misalignment_functor.mag_values = mag_values;
    misalignment_functor.m = m_misalignment;
    misalignment_functor.n = n_misalignment;

    Eigen::LevenbergMarquardt<mag_cal_misalignment_functor, double> lm_misalignment(misalignment_functor);
    lm_misalignment.minimize(x_misalignment);

    A << x_misalignment(0), x_misalignment(1), x_misalignment(2),
         x_misalignment(1), x_misalignment(3), x_misalignment(4),
         x_misalignment(2), x_misalignment(4), x_misalignment(5);
        
    b << x_misalignment(6),
         x_misalignment(7),
         x_misalignment(8);

    c = x_misalignment(9);

    ROS_INFO("Mag misalignment data values");
    std::cout << "A =\n" << A << std::endl;
    std::cout << "b =\n" << b << std::endl;
    std::cout << "c =\n" << c << std::endl;

    alpha = 1 / (0.25 * b.transpose() * A.inverse() * b - c);

    aux = alpha * A;
    Ds_0 = aux.llt().matrixL();     // get lower matrix from cholesky decomposition
    
    oh_0 = -0.5 * A.inverse() * b;

     return 0;
}

int mag_calibration_init(Eigen::MatrixXd mag_values, Eigen::VectorXd &output_values)
{
     return 0;
}