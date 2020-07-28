/**
 * @file accel_calibration.cpp
 * @author Francisco A. Dominguez (dominguezfranciscoa@gmail.com)
 * @brief
 * @version 0.1
 * @date 2020-06-20
 *
 * @copyright Copyright (c) 2020
 *
 */

#include "accel_calibration.h"

using namespace ramon_calibration;

/**
 * @fn AccelCalibration
 * @brief Accelerometer calibration constructor.
 * @param
 *  none.
 * @return
 *  none.
 */
AccelCalibration::AccelCalibration()
{
    Eigen::Matrix3f T_init;
    Eigen::Matrix3f K_init;
    Eigen::Vector3f b_init;
    Eigen::Vector3f noise;

    // axis missalignments (init values)
    T_init << 1, -0.01, 0.01,
              0,  1  , -0.01,
              0,  0  ,  1  ;

    // axis scales (init values)
    K_init << 1, 0, 0,
              0, 1, 0,
              0, 0, 1;

    // biases (init values)
    b_init << 0.01,
              0.01,
              0.01;

    // Set initial values
    this->setInitalValues(T_init, K_init, b_init);

    // Disable noise usage, sending a zero-norm vector
    noise << 0,
             0,
             0;    
    this->setNoise(noise);

    n_ = 9;
    calibrated_ = false;
}

/**
 * @fn AccelCalibration
 * @brief Accelerometer calibration constructor.
 * @param
 *  noise - Measurement noise vector in (x,y,z)
 * @return
 *  none.
 */
AccelCalibration::AccelCalibration(Eigen::Vector3f noise)
{
    Eigen::Matrix3f T_init;
    Eigen::Matrix3f K_init;
    Eigen::Vector3f b_init;

    // axis missalignments (init values)
    T_init << 1, -0.01, 0.01,
              0,  1  , -0.01,
              0,  0  ,  1  ;

    // axis scales (init values)
    K_init << 1, 0, 0,
              0, 1, 0,
              0, 0, 1;

    // biases (init values)
    b_init << 0.01,
              0.01,
              0.01;

    // Set initial values
    this->setInitalValues(T_init, K_init, b_init);

    // Set noise value and enable its usage
    this->setNoise(noise);

    n_ = 9;
    calibrated_ = false;
}

/**
 * @fn AccelCalibration
 * @brief Accelerometer calibration constructor.
 * @param
 *  T_init - Init values of Transformation matrix
 *  K_init - Init values of Scaling matrix
 *  b_init - Init values of bias vector
 * @return
 *  none.
 */
AccelCalibration::AccelCalibration(Eigen::Matrix3f T_init, Eigen::Matrix3f K_init, Eigen::Vector3f b_init)
{
    Eigen::Vector3f noise;

    // Set initial values
    this->setInitalValues(T_init, K_init, b_init);

    // Disable noise usage, sending a zero-norm vector
    noise << 0,
             0,
             0;    
    this->setNoise(noise);

    n_ = 9;
    calibrated_ = false;
}

/**
 * @fn AccelCalibration
 * @brief Accelerometer calibration constructor.
 * @param
 *  T_init - Init values of Transformation matrix
 *  K_init - Init values of Scaling matrix
 *  b_init - Init values of bias vector
 *  noise - Measurement noise vector in (x,y,z)
 * @return
 *  none.
 */
AccelCalibration::AccelCalibration(Eigen::Matrix3f T_init, Eigen::Matrix3f K_init, Eigen::Vector3f b_init, Eigen::Vector3f noise)
{
    // Set initial values
    this->setInitalValues(T_init, K_init, b_init);
    
    // Set noise value and enable its usage
    this->setNoise(noise);

    n_ = 9;
    calibrated_ = false;
}

/**
 * @fn ~AccelCalibration
 * @brief Accelerometer calibration destructor.
 * @param
 *  none
 * @return
 *  none.
 */
AccelCalibration::~AccelCalibration()
{

}

/**
 * @fn calibrate
 * @brief Function to calibrate the accelerometer
 * @param
 *  accel_values - Raw acceleration values
 * @return
 *  Eigen::LevenbergMarquardtSpace::Status - Status of minimization process
 */
Eigen::LevenbergMarquardtSpace::Status AccelCalibration::calibrate(const Eigen::Ref<const Eigen::MatrixXf> accel_values)
{
    Eigen::VectorXf x(n_);   // Values to get by LM algorithm
    accel_cal_functor functor(accel_values, accel_values.rows(), n_);
    Eigen::LevenbergMarquardtSpace::Status ret;
    Eigen::NumericalDiff<accel_cal_functor> numDiff(functor);
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<accel_cal_functor>, float> lm(numDiff);
    // accel_cal_functor functor;
    // Eigen::LevenbergMarquardtSpace::Status ret;
    // Eigen::LevenbergMarquardt<accel_cal_functor, float> lm(functor);

    // Init functor
    // functor.accel_values = accel_values;
    // functor.m = accel_values.rows();
    // functor.n = n_;

    // Set x variables to minimize
    x(0) = alpha_yz_;
    x(1) = alpha_zy_;
    x(2) = alpha_zx_;
    x(3) = s_x_;
    x(4) = s_y_;
    x(5) = s_z_;
    x(6) = b_x_;
    x(7) = b_y_;
    x(8) = b_z_;

    // Minimize
    ret = lm.minimize(x);

    T_ << 1, -x(0) ,  x(1) ,
          0,   1   , -x(2) ,
          0,   0   ,   1   ;

    K_ << x(3),  0  ,  0   ,
            0 , x(4),  0   ,
            0 ,   0  , x(5);

    b_ << x(6),
          x(7),
          x(8);
    
    calibrated_ = true;

    return ret;
}

/**
 * @fn getBias
 * @brief Obtain the bias vector
 * @param
 *  b - Estimated bias vector to be obtained
 * @return
 *  int - if the accelerometer was previously calibrated, returns 0, otherwise, -1
 */
int AccelCalibration::getBias(Eigen::Ref<Eigen::Vector3f> b)
{
    int out;
    if(calibrated_)
    {
        out = 0;
        b << b_;
    }
    else
    {
        out = -1;
    }
    return out;    
}

/**
 * @fn getParams
 * @brief Obtain the estimated parameters based on calibration
 * @param
 *  T - Estimated Transform matrix to be obtained
 *  K - Estimated Scaling matrix to be obtained
 *  b - Estimated bias vector to be obtained
 * @return
 *  int - if the accelerometer was previously calibrated, returns 0, otherwise, -1
 */
int AccelCalibration::getParams(Eigen::Ref<Eigen::Matrix3f> T, Eigen::Ref<Eigen::Matrix3f> K, Eigen::Ref<Eigen::Vector3f> b)
{
    int ret = -1;
    if(calibrated_)
    {
        ret = 0;
        getT(T);
        getK(K);
        getBias(b);
    }
    return ret;
}

/**
 * @fn getK
 * @brief Obtain the Scaling matrix
 * @param
 *  K - Estimated Scaling matrix to be obtained
 * @return
 *  int - if the accelerometer was previously calibrated, returns 0, otherwise, -1
 */
int AccelCalibration::getK(Eigen::Ref<Eigen::Matrix3f> K)
{
    int out;
    if(calibrated_)
    {
        out = 0;
        K << K_;
    }
    else
    {
        out = -1;
    }
    return out;    
}

/**
 * @fn getT
 * @brief Obtain the Transform matrix
 * @param
 *  T - Estimated Transform matrix to be obtained
 * @return
 *  int - if the accelerometer was previously calibrated, returns 0, otherwise, -1
 */
int AccelCalibration::getT(Eigen::Ref<Eigen::Matrix3f> T)
{
    int out;
    if(calibrated_)
    {
        out = 0;
        T << T_;
    }
    else
    {
        out = -1;
    }
    return out;    
}

/**
 * @fn printCalibratedValues
 * @brief Print corrected values if calibrated
 * @param
 *  none
 * @return
 *  int - if the accelerometer was previously calibrated, returns 0, otherwise, -1
 */
int AccelCalibration::printCalibratedValues(void)
{
    int ret = -1;
    if(calibrated_)
    {
        ret = 0;
        std::cout << "T_a =\n" << T_ << std::endl;
        std::cout << "K_a =\n" << K_ << std::endl;
        std::cout << "b_a =\n" << b_ << std::endl;
    }
    return ret;
}

/**
 * @fn setInitialValues
 * @brief Set the initial values for the LM algorithm
 * @param
 *  T_init - Transform matrix initial values
 *  K_init - Scaling matrix initial values
 *  b_init - bias vector initial values
 * @return
 *  none.
 */
void AccelCalibration::setInitalValues(Eigen::Matrix3f T_init, Eigen::Matrix3f K_init, Eigen::Vector3f b_init)
{
    // axis missalignments (init values)
    alpha_yz_ = -T_init(0,1);
    alpha_zy_ = T_init(0,2);
    alpha_zx_ = -T_init(1,2);

    // axis scales (init values)
    s_x_ = K_init(0,0);
    s_y_ = K_init(1,1);
    s_z_ = K_init(2,2);

    // biases (init values)

    b_x_ = b_init(0);
    b_y_ = b_init(1);
    b_z_ = b_init(2);
}

/**
 * @fn setNoise
 * @brief Set the measurement noise values
 * @param
 *  noise - measurement noise values
 * @return
 *  none.
 */
void AccelCalibration::setNoise(Eigen::Vector3f noise)
{
    noise_ = noise;
    if(noise.norm() == 0.0)
        use_noise_ = false;
    else
        use_noise_ = true;
}

/**
 * @fn correctValues
 * @brief Given a set of accelerometer measurements, correct those values based on calibration
 * @param
 *  accel_values - accelerometer values to be calibrated
 *  accel_values_corr - corrected accelerometer values
 * @return
 *  int - if the accelerometer was previously calibrated, returns 0, otherwise, -1
 */
int AccelCalibration::correctValues(const Eigen::Ref<const Eigen::MatrixX3f> accel_values, Eigen::Ref<Eigen::MatrixX3f> accel_values_corr)
{
    int ret = -1;
    if(calibrated_)
    {
        ret = 0;
        accel_values_corr = (T_ * K_ * (accel_values.rowwise() + b_.transpose()).transpose()).transpose();
    }

    return ret;
}