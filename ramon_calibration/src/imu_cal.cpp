#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <Eigen/Eigen>
#include <unsupported/Eigen/NonLinearOptimization>

#include "gyro_cal.h"
#include "accel_cal.h"
#include "static_detector.h"


#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

int main (int argc, char** argv)
{
    ros::init (argc, argv, "imu_calibration");    
    rosbag::Bag bag;
	Eigen::MatrixXf accel_values, accel_values_corr, gyro_values, gyro_values_corr;
    Eigen::VectorXf output_accel_values(9), output_gyro_values(12);
    Eigen::Matrix3f T_a, K_a, T_g, K_g;
    Eigen::Vector3f b_a, b_g;
    Eigen::MatrixXf static_data_aux;
    // std::ofstream file("test.txt");

    ros::NodeHandle nh("~");

    std::string bag_file;
    std::string topic_name;
    float data_rate;
    float t_init;
    float t_w;
    if(!nh.getParam("bag_file", bag_file))
    {
        ROS_ERROR("Bag file needed");
        return -1;
    }
    if(!nh.getParam("topic_name", topic_name))
    {
        ROS_ERROR("Topic name needed");
        return -1;
    }    
    if(!nh.getParam("data_rate", data_rate))
    {
        data_rate = 0.005;
    }
    if(!nh.getParam("t_init", t_init))
    {
        ROS_ERROR("Tinit needed");
        return -1;
    }
    if(!nh.getParam("t_w", t_w))
    {
        ROS_ERROR("t_w needed");
        return -1;
    }
    ROS_INFO("\nBag file: %s", bag_file.c_str());
    ROS_INFO("\nTopic name: %s", topic_name.c_str());
    ROS_INFO("\nData rate: %f", data_rate);
    ROS_INFO("\nTinit: %f", t_init);
    ROS_INFO("\ntw: %f", t_w);

    // Load bagfile
    bag.open(bag_file, rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(topic_name);
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    ros::Time::init();
    int j = 0;

    // Tinit in samples
    int tinit_samples = int(t_init / data_rate);
    ROS_INFO("\nTinit_samples: %d", tinit_samples);

    // Tw in samples
    int tw_samples = int(t_w / data_rate);

    // Initial static detector coeff    
    float e_init;
    // Actual static detector coeff
    float e;

    // Get values of gyro and accel
    accel_values.resize(bag.getSize(),3);
    gyro_values.resize(bag.getSize(),3);
    foreach(rosbag::MessageInstance const mm, view)
    {
        sensor_msgs::Imu::ConstPtr i = mm.instantiate<sensor_msgs::Imu>();
        if (i != NULL)
        {
            accel_values(j, 0) = i->linear_acceleration.x;
            accel_values(j, 1) = i->linear_acceleration.y;
            accel_values(j, 2) = i->linear_acceleration.z;
            gyro_values(j,0) = i->angular_velocity.x;
            gyro_values(j,1) = i->angular_velocity.y;
            gyro_values(j,2) = i->angular_velocity.z;
            j++;
        }
    }
    accel_values.conservativeResize(j, 3);
    accel_values_corr.resize(j, 3);
    gyro_values.conservativeResize(j, 3);
    gyro_values_corr.resize(j, 3);

    // Normalize accel values
    accel_values = accel_values / -9.80665;


    // Plot the accelerometer values
    Eigen::VectorXf v1 = accel_values.col(0);
    std::vector<float> v2, t(accel_values.rows());
    std::iota(t.begin(), t.end(), 0);
    float myconstant{0.005};
    std::transform(t.begin(), t.end(), t.begin(), [&myconstant](auto& c){return c*myconstant;});
    v2.resize(v1.size());
    Eigen::VectorXf::Map(&v2[0], v1.size()) = v1;
    plt::named_plot("X Axis", t, v2);
    
    v1 = accel_values.col(1);    
    v2.resize(v1.size());
    Eigen::VectorXf::Map(&v2[0], v1.size()) = v1;
    plt::named_plot("Y Axis", t, v2);
    
    v1 = accel_values.col(2);   
    v2.resize(v1.size());
    Eigen::VectorXf::Map(&v2[0], v1.size()) = v1;
    plt::named_plot("Z Axis", t, v2);

    plt::ylabel("Linear Acceleration [m/s^2]");
    plt::xlabel("Time [s]");
    plt::title("Accelerometer readings");
    plt::legend();
    plt::show();

    // Get init values, based on init_samples
    get_static_detector_coeff(accel_values.block(0,0,tinit_samples,3), e_init);

    ROS_INFO("\nStatic detector coeff init: %f", e_init);

    // get_static_intervals_prom(accel_values, tw_samples, static_intervals_prom);

    /*
    // Calibrate accelerometer based on the static detections
    accel_calibration(accel_values, output_accel_values);

    T_a << 1, -output_accel_values(0),  output_accel_values(1),
           0,           1            , -output_accel_values(2),
           0,           0            ,            1           ;  

    K_a << output_accel_values(3),          0            ,          0             ,
                    0            , output_accel_values(4),          0             ,
                    0            ,          0            ,  output_accel_values(5);
    
    b_a << output_accel_values(6),
           output_accel_values(7),
           output_accel_values(8);         

    // Get accelerometer corrected values
    for(int i=0; i<j; i++)
    {
        accel_values_corr.row(i) = (T_a * K_a * (accel_values.row(i).transpose() + b_a)).transpose();
    }

    //Calibrate gyroscope based on corrected accelerometer 
    gyro_calibration(accel_values_corr, gyro_values, 5.0, output_gyro_values);

    T_g <<   1  , -output_gyro_values(0),  output_gyro_values(1),
            output_gyro_values(3),   1  , -output_gyro_values(2),
            -output_gyro_values(4),  output_gyro_values(5),   1  ;  

    K_g << output_gyro_values(6),  0  ,   0  ,
            0  , output_gyro_values(7),   0  ,
            0  ,  0  ,  output_gyro_values(8);
    
    b_g << output_gyro_values(9),
            output_gyro_values(10),
            output_gyro_values(11);

    for(int i=0; i<j; i++)
    {
        gyro_values_corr.row(i) = (T_g * K_g * (gyro_values.row(i).transpose() + b_g)).transpose();
    }

    std::cout << "Accel data values" << std::endl;
    std::cout <<  output_accel_values << std::endl;
    std::cout << "Gyro data values" << std::endl;
    std::cout <<  output_gyro_values << std::endl;
    */

    return 0;
}