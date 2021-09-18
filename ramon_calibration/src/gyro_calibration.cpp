/**
 * @file gyro_calibration.cpp
 * @author Francisco A. Dominguez (dominguezfranciscoa@gmail.com)
 * @brief
 * @version 0.1
 * @date 2020-06-20
 *
 * @copyright Copyright (c) 2020
 *
 */
#include "gyro_calibration.h"
#include <ros/ros.h>

using namespace ramon_calibration;

/**
 * @fn GyroCalibration
 * @brief Gyroscope calibration constructor.
 * @param
 *  none.
 * @return
 *  none.
 */
GyroCalibration::GyroCalibration()
{
    Eigen::Matrix3d T_init;
    Eigen::Matrix3d K_init;
    Eigen::Vector3d noise;

    // axis missalignments (init values)
    T_init <<   1  , -0.01,  0.01,
               0.01,   1  , -0.01,
              -0.01,  0.01,   1  ;

    // axis scales (init values)
    K_init << 1, 0, 0,
              0, 1, 0,
              0, 0, 1;

    // Set initial values
    this->setInitalValues(T_init, K_init);

    // Disable noise usage, sending a zero-norm vector
    noise << 0,
             0,
             0;    
    this->setNoise(noise);

    n_ = 9;
    calibrated_ = false;
}

/**
 * @fn GyroCalibration
 * @brief Gyroscope calibration constructor.
 * @param
 *  noise - Measurement noise vector in (x,y,z)
 * @return
 *  none.
 */
GyroCalibration::GyroCalibration(Eigen::Vector3d noise)
{
    Eigen::Matrix3d T_init;
    Eigen::Matrix3d K_init;

    // axis missalignments (init values)
    T_init <<   1  , -0.01,  0.01,
               0.01,   1  , -0.01,
              -0.01,  0.01,   1  ;

    // axis scales (init values)
    K_init << 1, 0, 0,
              0, 1, 0,
              0, 0, 1;

    // Set initial values
    this->setInitalValues(T_init, K_init);

    // Set noise value and enable its usage
    this->setNoise(noise);

    n_ = 9;
    calibrated_ = false;
}

/**
 * @fn GyroCalibration
 * @brief Gyroscope calibration constructor.
 * @param
 *  T_init - Init values of Transformation matrix
 *  K_init - Init values of Scaling matrix
 *  b_init - Init values of bias vector
 * @return
 *  none.
 */
GyroCalibration::GyroCalibration(Eigen::Matrix3d T_init, Eigen::Matrix3d K_init)
{
    Eigen::Vector3d noise;

    // Set initial values
    this->setInitalValues(T_init, K_init);

    // Disable noise usage, sending a zero-norm vector
    noise << 0,
             0,
             0;    
    this->setNoise(noise);

    n_ = 9;
    calibrated_ = false;
}

/**
 * @fn GyroCalibration
 * @brief Gyroscope calibration constructor.
 * @param
 *  T_init - Init values of Transformation matrix
 *  K_init - Init values of Scaling matrix
 *  b_init - Init values of bias vector
 *  noise - Measurement noise vector in (x,y,z)
 * @return
 *  none.
 */
GyroCalibration::GyroCalibration(Eigen::Matrix3d T_init, Eigen::Matrix3d K_init, Eigen::Vector3d noise)
{
    // Set initial values
    this->setInitalValues(T_init, K_init);
    
    // Set noise value and enable its usage
    this->setNoise(noise);

    n_ = 9;
    calibrated_ = false;
}

/**
 * @fn ~GyroCalibration
 * @brief Gyroscope calibration destructor.
 * @param
 *  none
 * @return
 *  none.
 */
GyroCalibration::~GyroCalibration()
{

}

/**
 * @fn calibrate
 * @brief Function to calibrate the Gyroscope
 * @param
 *  accel_values - Calibrated acceleration values
 *  gyro_values - Raw angular velocity values
 *  static_intervals - Matrix containing start(col0)-end(col1) index of each static interval(rows)
 *  tinit_samples - Init samples used to get gyroscope bias (got from Allan variance)
 *  dt - Iime in seconds between each measurement
 * @return
 *  Eigen::LevenbergMarquardtSpace::Status - Status of minimization process
 */
Eigen::LevenbergMarquardtSpace::Status GyroCalibration::calibrate(const Eigen::Ref<const Eigen::MatrixX3d> accel_values, 
                                                                  const Eigen::Ref<const Eigen::MatrixX3d> gyro_values, 
                                                                  const Eigen::Ref<const Eigen::MatrixX2i> static_times,  
                                                                  int tinit_samples, double dt)
{
    Eigen::VectorXd x(n_);   // Values to get by LM algorithm
    double dt_factor = 2.0;
    Eigen::Vector4d q_init;
    q_init << 1,
              0,
              0,
              0;

    // Block of size (p,q), starting at (i,j) => matrix.block(i,j,p,q);
    b_ = gyro_values.block(0, 0, tinit_samples, 3).colwise().mean().transpose();

    gyro_cal_functor functor(accel_values, gyro_values, static_times, dt, dt_factor, accel_values.rows(), n_, b_, noise_, q_init);
    Eigen::LevenbergMarquardtSpace::Status ret;
    Eigen::NumericalDiff<gyro_cal_functor> numDiff(functor);
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<gyro_cal_functor>, double> lm(numDiff);

    // lm.parameters.maxfev = 100;
    // lm.parameters.xtol = 1.0e-5;
    // lm.parameters.ftol = 1.0e-5;

    // Set x variables to minimize
    x(0) = gamma_yz_;
    x(1) = gamma_zy_;
    x(2) = gamma_zx_;
    x(3) = gamma_xz_;
    x(4) = gamma_xy_;
    x(5) = gamma_yx_;
    x(6) = s_x_;
    x(7) = s_y_;
    x(8) = s_z_;
    
    // Minimize
    ret = lm.minimize(x);

    T_ <<  1  , -x(0),  x(1),
          x(3),   1  , -x(2),
         -x(4),  x(5),   1  ;

    K_ << x(6),  0  ,  0   ,
            0 , x(7),  0   ,
            0 ,   0  , x(8);
    
    calibrated_ = true;

    return ret;
}

/**
 * @fn getBias
 * @brief Obtain the bias vector
 * @param
 *  b - Estimated bias vector to be obtained
 * @return
 *  int - if the Gyroscope was previously calibrated, returns 0, otherwise, -1
 */
int GyroCalibration::getBias(Eigen::Ref<Eigen::Vector3d> b)
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
 *  int - if the Gyroscope was previously calibrated, returns 0, otherwise, -1
 */
int GyroCalibration::getParams(Eigen::Ref<Eigen::Matrix3d> T, Eigen::Ref<Eigen::Matrix3d> K, Eigen::Ref<Eigen::Vector3d> b)
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
 *  int - if the Gyroscope was previously calibrated, returns 0, otherwise, -1
 */
int GyroCalibration::getK(Eigen::Ref<Eigen::Matrix3d> K)
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
 *  int - if the Gyroscope was previously calibrated, returns 0, otherwise, -1
 */
int GyroCalibration::getT(Eigen::Ref<Eigen::Matrix3d> T)
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
 *  int - if the gyroscope was previously calibrated, returns 0, otherwise, -1
 */
int GyroCalibration::printCalibratedValues(void)
{
    int ret = -1;
    if(calibrated_)
    {
        ret = 0;
        std::cout << "T_g =\n" << T_ << std::endl;
        std::cout << "K_g =\n" << K_ << std::endl;
        std::cout << "b_g =\n" << b_ << std::endl;
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
void GyroCalibration::setInitalValues(Eigen::Matrix3d T_init, Eigen::Matrix3d K_init)
{
    // axis missalignments (init values)
    gamma_yz_ = -T_init(0,1);
    gamma_zy_ = T_init(0,2);
    gamma_zx_ = -T_init(1,2);

    gamma_xz_ = T_init(1,0);
    gamma_xy_ = -T_init(2,0);
    gamma_yx_ = T_init(2,1);

    // axis scales (init values)
    s_x_ = K_init(0,0);
    s_y_ = K_init(1,1);
    s_z_ = K_init(2,2);
}

/**
 * @fn setNoise
 * @brief Set the measurement noise values
 * @param
 *  noise - measurement noise values
 * @return
 *  none.
 */
void GyroCalibration::setNoise(Eigen::Vector3d noise)
{
    noise_ = noise;
    if(noise.norm() == 0.0)
        use_noise_ = false;
    else
        use_noise_ = true;
}

/**
 * @fn correctValues
 * @brief Given a set of Gyroscope measurements, correct those values based on calibration
 * @param
 *  accel_values - Gyroscope values to be calibrated
 *  accel_values_corr - corrected Gyroscope values
 * @return
 *  int - if the Gyroscope was previously calibrated, returns 0, otherwise, -1
 */
int GyroCalibration::correctValues(const Eigen::Ref<const Eigen::MatrixXd> accel_values, Eigen::Ref<Eigen::MatrixXd> accel_values_corr)
{
    int i, ret = -1;
    if(calibrated_)
    {
        ret = 0;
        for(i = 0; i < accel_values.rows(); i++)
        {
            accel_values_corr.row(i) = (T_ * K_ * (accel_values.row(i).transpose() + b_)).transpose();
        }
    }
    return ret;
}