/**
 * @file imu_calibration.cpp
 * @author Francisco A. Dominguez (dominguezfranciscoa@gmail.com)
 * @brief
 * @version 0.1
 * @date 2020-06-20
 *
 * @copyright Copyright (c) 2020
 *
 */

#include "imu_calibration.h"

using namespace ramon_calibration;

/**
 * @fn IMUCalibration
 * @brief IMU calibration constructor.
 * @param
 *  none.
 * @return
 *  none.
 */
IMUCalibration::IMUCalibration(): nh_("~")
{
    this->init();   
}

/**
 * @fn ~IMUCalibration
 * @brief IMU calibration destructor.
 * @param
 *  none.
 * @return
 *  none.
 */
IMUCalibration::~IMUCalibration()
{
    
}

/**
 * @fn calibrate
 * @brief Calibrate IMU.
 * @param
 *  none.
 * @return
 *  int 0 : ok
 *      -1: it is necessary to call the function start() before
 *      -2: Accelerometer calibration error
 *      -3: Gyroscope calibration error
 */
int IMUCalibration::calibrate(void)
{
    int accel_cal_ret;
    int gyro_cal_ret;
    int ret = -1;

    if(start_)
    {
        ret = 0;
        ROS_INFO("Starting accelerometer calibration...");
        accel_cal_ret = accel_cal_.calibrate(accel_values_prom_);
        ROS_INFO("Done.");
        ROS_INFO("Accelerometer calibration return value: %d", accel_cal_ret);

        accel_cal_.correctValues(accel_values_, accel_values_corr_);
        ROS_INFO("Accelerometer calibration values");
        if(accel_cal_.printCalibratedValues() < 0)
        {
            ret = -2;        
        }
        else
        {
            ROS_INFO("Starting gyro calibration...");
            /* TODO: Finish */
            gyro_cal_ret = gyro_cal_.calibrate(accel_values_corr_, gyro_values_, static_timestep_, tinit_samples_, data_rate_);
            ROS_INFO("Done.");
            ROS_INFO("Gyroscope calibration return value: %d", accel_cal_ret);

            ROS_INFO("Gyroscope calibration values");
            if(gyro_cal_.printCalibratedValues() < 0)
            {
                ret = -3;
            }
        }
    }
    return ret;
}

/**
 * @fn correctAccelValues
 * @brief Correct accelerometer values with calibration.
 * @param
 *  accel_in Input data to calibrate.
 *  accel_out Output data calibrated.
 * @return
 *  int 0: ok, otherwise, calibration not performed
 */
int IMUCalibration::correctAccelValues(const Eigen::Ref<const Eigen::MatrixXf> accel_in, Eigen::Ref<Eigen::MatrixXf> accel_out)
{
    this->accel_cal_.correctValues(accel_in, accel_out);
}

/**
 * @fn getStaticDetectorCoefficent
 * @brief Get the static detector coefficient from tinit data.
 * @param
 *  none.
 * @return
 *  int 0: ok.
 */
int IMUCalibration::getStaticDetectorCoefficient(void)
{
    Eigen::Vector3f var;
    if(this->getVariance(accel_values_.block(0, 0, tinit_samples_, 3), var) < 0)
    {
        return -1;
    }

    e_init_ = var.norm();

    return 0;
}

/**
 * @fn getStaticDetectorCoefficent
 * @brief Get the static intervals based on the static detector.
 * @param
 *  none.
 * @return
 *  int 0: ok
 */
/* TODO: Terminar */
int IMUCalibration::getStaticIntervals(void)
{
    // int count_tw = int(accel_values_.rows() / t_w_samples_);
    Eigen::Vector3f var_aux;
    int i = tinit_samples_;
    bool static_interval = false;
    int index = 0;
    int prom_index = 0;
    int intervals = 0;
    int static_time_index = 0;
    int static_time_init = -1;

    static_data_ = Eigen::VectorXf::Constant(accel_values_.rows(), 0);
    static_timestep_.resize(accel_values_.rows(), 2);
    accel_values_prom_.resize(accel_values_.rows(), 3);

    while(i < accel_values_.rows() - t_w_samples_)
    {
        if(getVariance(accel_values_.block(i, 0, t_w_samples_, 3), var_aux) >= 0)
        {
            if(var_aux.norm() < e_init_k_ * e_init_)
            {
                static_interval = true;
                if(static_time_init < 0)
                {
                    static_time_init = i;
                }
                index = i + 1;
                static_data_.segment(i, t_w_samples_) = Eigen::VectorXf::Constant(t_w_samples_, 2);
                i += t_w_samples_;
                intervals += t_w_samples_;
            }
            else
            {
                if(static_interval)
                {
                    static_interval = false;
                    static_timestep_(static_time_index, 0) = static_time_init;
                    static_timestep_(static_time_index, 1) = i;
                    static_time_index++;
                    static_time_init = -1;
                    /* TODO: Ver que hacer */
                    accel_values_prom_.row(prom_index) = accel_values_.block(index, 0, i - index, 3).colwise().mean();
                    prom_index++;
                }
                static_data_(i) = 0.0;
                i++;
            }
        }
    }
    accel_values_prom_.conservativeResize(prom_index,3);
    static_timestep_.conservativeResize(static_time_index,2);

    return 0;
}

/**
 * @fn getVariance
 * @brief Get variance from input matrix.
 * @param
 *  values Input matrix of three columns
 *  variance Output variance of each column of the input matrix.
 * @return
 *  int 0: ok.
 */
int IMUCalibration::getVariance(const Eigen::Ref<const Eigen::MatrixXf> values, Eigen::Ref<Eigen::Vector3f> variance)
{
    Eigen::MatrixXf mean_values, mean_matrix(values.rows(),values.cols());

    if(values.rows() <= 1)
    {
        return -1;
    }
    mean_values = values.colwise().mean();

    mean_matrix << Eigen::MatrixXf::Constant(values.rows(),1,mean_values(0)), 
                   Eigen::MatrixXf::Constant(values.rows(),1,mean_values(1)), 
                   Eigen::MatrixXf::Constant(values.rows(),1,mean_values(2));
    variance = ((values - mean_matrix).array() * (values - mean_matrix).array()).colwise().sum().transpose();
    variance /= (values.rows() - 1);

    return 0;
}

/**
 * @fn init
 * @brief Initializes the private variables of the class.
 * @param
 *  none.
 * @return
 *  int 0: ok.
 */
int IMUCalibration::init(void)
{
    if(!nh_.getParam("bag_file", bag_file_))
    {
        ROS_ERROR("Bag file needed");
        return -1;
    }
    if(!nh_.getParam("topic_name", topic_name_))
    {
        ROS_ERROR("Topic name needed");
        return -1;
    }    
    if(!nh_.getParam("data_rate", data_rate_))
    {
        data_rate_ = 0.005;
    }
    if(!nh_.getParam("e_init_k", e_init_k_))
    {
        e_init_k_ = 2.0;
    }
    if(!nh_.getParam("t_init", t_init_))
    {
        ROS_ERROR("Tinit needed");
        return -1;
    }
    if(!nh_.getParam("t_w", t_w_))
    {
        ROS_ERROR("t_w needed");
        return -1;
    }
    if(!nh_.getParam("gravity", gravity_))
    {
        gravity_ = GRAVITY;
    }   
    if(!nh_.getParam("plot_data", plot_data_))
    {
        plot_data_ = false;
    }   
    tinit_samples_ = int(t_init_ / data_rate_);
    t_w_samples_ = int(t_w_ / data_rate_);
    ros::Time::init();

    ROS_INFO("\nBag file: %s", bag_file_.c_str());
    ROS_INFO("\nTopic name: %s", topic_name_.c_str());
    ROS_INFO("\nData rate: %f", data_rate_);
    ROS_INFO("\nNoise floor k: %f", e_init_k_);
    ROS_INFO("\nTinit: %f", t_init_);
    ROS_INFO("\nTinit_samples: %d", tinit_samples_);
    ROS_INFO("\ntw: %f", t_w_);
    ROS_INFO("\ntw_samples: %d", t_w_samples_);
    ROS_INFO("\ngravity: %f", gravity_);

    return 0;
}

/**
 * @fn plotAccel
 * @brief Plot accelerometer data with static detector.
 * @param
 *  none
 * @return
 *  int 0: ok, otherwise indicates that it is necessary to call the function start() before
 */
int IMUCalibration::plotAccel(void)
{
    std::string x_label("Time [s]");
    std::string y_label("Linear acceleration [m/s^2]");
    std::string title("Accelerometer readings with static detector");
    int ret = -1;

    if(start_)
    {
        ret = 0;
        this->plotData(accel_values_, static_data_, data_rate_, x_label, y_label, title);
    }

    return ret;
}

/**
 * @fn plotData
 * @brief Plot data values with or without the static detector.
 * @param
 *  data_values Data values
 *  data_rate Time step between each sample
 * @return
 *  int 0: ok.
 */
void IMUCalibration::plotData(const Eigen::Ref<const Eigen::MatrixXf> data_values, float data_rate, std::string x_label, std::string y_label, std::string title)
{
    // Plot the values
    Eigen::VectorXf v1 = data_values.col(0);
    std::vector<float> v2, t(data_values.rows());
    std::iota(t.begin(), t.end(), 0);

    std::transform(t.begin(), t.end(), t.begin(), [&data_rate](auto& c){return c*data_rate;});
    v2.resize(v1.size());
    Eigen::VectorXf::Map(&v2[0], v1.size()) = v1;
    plt::named_plot("X axis", t, v2);
    
    v1 = data_values.col(1);    
    v2.resize(v1.size());
    Eigen::VectorXf::Map(&v2[0], v1.size()) = v1;
    plt::named_plot("Y axis", t, v2);
    
    v1 = data_values.col(2);   
    v2.resize(v1.size());
    Eigen::VectorXf::Map(&v2[0], v1.size()) = v1;
    plt::named_plot("Z axis", t, v2);

    plt::ylabel(y_label);
    plt::xlabel(x_label);
    plt::title(title);
    
    plt::legend();
    // plt::ion();
    plt::show();
    plt::close();
}

/**
 * @fn plotData
 * @brief Plot data values with or without the static detector.
 * @param
 *  data_values Data values
 *  points_status Static detector values
 *  data_rate Time step between each sample
 * @return
 *  int 0: ok.
 */
void IMUCalibration::plotData(const Eigen::Ref<const Eigen::MatrixXf> data_values, const Eigen::Ref<const Eigen::VectorXf> points_status, float data_rate, std::string x_label, std::string y_label, std::string title)
{
    // Plot the values
    Eigen::VectorXf v1 = data_values.col(0);
    std::vector<float> v2, t(data_values.rows());
    std::iota(t.begin(), t.end(), 0);

    std::transform(t.begin(), t.end(), t.begin(), [&data_rate](auto& c){return c*data_rate;});
    v2.resize(v1.size());
    Eigen::VectorXf::Map(&v2[0], v1.size()) = v1;
    plt::named_plot("X axis", t, v2);
    
    v1 = data_values.col(1);    
    v2.resize(v1.size());
    Eigen::VectorXf::Map(&v2[0], v1.size()) = v1;
    plt::named_plot("Y axis", t, v2);
    
    v1 = data_values.col(2);   
    v2.resize(v1.size());
    Eigen::VectorXf::Map(&v2[0], v1.size()) = v1;
    plt::named_plot("Z axis", t, v2);

    v1 = points_status;   
    v2.resize(v1.size());
    Eigen::VectorXf::Map(&v2[0], v1.size()) = v1;
    plt::named_plot("Static detector", t, v2, "k-");

    plt::ylabel(y_label);
    plt::xlabel(x_label);
    plt::title(title);

    plt::legend();
    // plt::ion();
    plt::show();
    plt::close();
}

/**
 * @fn plotGyro
 * @brief Plot gyroscope data.
 * @param
 *  none
 * @return
 *  int 0: ok, otherwise indicates that it is necessary to call the function start() before
 */
int IMUCalibration::plotGyro(void)
{
    std::string x_label("Time [s]");
    std::string y_label("Angular velocity [rad/s]");
    std::string title("Gyroscope readings");
    int ret = -1;

    if(start_)
    {
        ret = 0;
        this->plotData(gyro_values_, data_rate_, x_label, y_label, title);
    }

    return ret;
}

/**
 * @fn readBag
 * @brief Read from bagfile and save data.
 * @param
 *  none.
 * @return
 *  int 0: ok.
 */
int IMUCalibration::readBag(void)
{
    rosbag::Bag bag;
    int j = 0;

    // Load bagfile
    bag.open(bag_file_, rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(topic_name_);
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    foreach(rosbag::MessageInstance const mm, view)
    {
        // Possible message types
        ramon_calibration::ImuWithMag::ConstPtr imumag = mm.instantiate<ramon_calibration::ImuWithMag>();
        sensor_msgs::Imu::ConstPtr imu = mm.instantiate<sensor_msgs::Imu>();

        accel_values_.resize(bag.getSize(),3);
        gyro_values_.resize(bag.getSize(),3);
        mag_values_.resize(bag.getSize(),3);
        if (imumag != NULL)
        {
            accel_values_(j, 0) = imumag->linear_acceleration.x;
            accel_values_(j, 1) = imumag->linear_acceleration.y;
            accel_values_(j, 2) = imumag->linear_acceleration.z;
            gyro_values_(j,0) = imumag->angular_velocity.x;
            gyro_values_(j,1) = imumag->angular_velocity.y;
            gyro_values_(j,2) = imumag->angular_velocity.z;
            mag_values_(j,0) = imumag->magnetic_field.x;
            mag_values_(j,1) = imumag->magnetic_field.y;
            mag_values_(j,2) = imumag->magnetic_field.z;
            j++;
        }
        else if (imu != NULL)
        {
            accel_values_(j, 0) = imu->linear_acceleration.x;
            accel_values_(j, 1) = imu->linear_acceleration.y;
            accel_values_(j, 2) = imu->linear_acceleration.z;
            gyro_values_(j,0) = imu->angular_velocity.x;
            gyro_values_(j,1) = imu->angular_velocity.y;
            gyro_values_(j,2) = imu->angular_velocity.z;
            j++;
        }
    }    
    accel_values_.conservativeResize(j, 3);
    accel_values_corr_.resize(j, 3);
    gyro_values_.conservativeResize(j, 3);
    gyro_values_corr_.resize(j, 3);
    mag_values_.conservativeResize(j,3);
    mag_values_corr_.resize(j,3);

    return 0;
}

/**
 * @fn start
 * @brief Get needed data from bagfile and parameters.
 * @param
 *  none.
 * @return
 *  int 0: ok.
 */
int IMUCalibration::start(void)
{

    // Read bagfile and load parameters.
    this->readBag();

    ROS_INFO("Bag read");
    // Normalize accel values
    accel_values_ = accel_values_ / gravity_;

    // Get static detector init value, based on init_samples
    this->getStaticDetectorCoefficient();
    ROS_INFO("\nStatic detector coeff init: %f", e_init_);

    // Get averaged static intervals
    this->getStaticIntervals();

    // Set start flag
    start_ = true;

    if(plot_data_)
    {
        // Plot acceleration data along with static detector
        this->plotAccel();
        // Plot gyroscope data
        this->plotGyro();
    }    
    return 0;
}