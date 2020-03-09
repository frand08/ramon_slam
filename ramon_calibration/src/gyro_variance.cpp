#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include "allan_variance.h"

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

int main (int argc, char** argv)
{
    ros::init (argc, argv, "gyro_variance");  
    rosbag::Bag bag;
	Eigen::MatrixXf gyro_values, gyro_allan_variance;
    std::ofstream file("test.txt");
  
    bag.open("/home/fdominguez/Documents/bagfiles/imu_data2.bag", rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string("/imu_data"));
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    ros::Time::init();
    int j = 0;
    
    gyro_values.resize(bag.getSize(),3);
    foreach(rosbag::MessageInstance const mm, view)
    {
        sensor_msgs::Imu::ConstPtr i = mm.instantiate<sensor_msgs::Imu>();
        if (i != NULL)
        {
            gyro_values(j,0) = i->angular_velocity.x;
            gyro_values(j,1) = i->angular_velocity.y;
            gyro_values(j,2) = i->angular_velocity.z;
            j++;
        }
    }
    gyro_values.conservativeResize(j, 3);

    allan_variance(gyro_values, gyro_allan_variance);
    
    // if (file.is_open())
    // {
    //     file << "Here are the values of the gyro Allan variance x values:\n" << gyro_allan_variance.col(0) << '\n';
    //     file << "Here are the values of the gyro Allan variance y values:\n" << gyro_allan_variance.col(1) << '\n';
    //     file << "Here are the values of the gyro Allan variance z values:\n" << gyro_allan_variance.col(2) << '\n';
    // }

    // Plot the outputs

    Eigen::VectorXf v1 = gyro_allan_variance.col(0);
    std::vector<float> v2, t(gyro_allan_variance.rows());
    std::iota(t.begin(), t.end(), 0);
    float myconstant{0.005};
    std::transform(t.begin(), t.end(), t.begin(), [&myconstant](auto& c){return c*myconstant;});
    
    v2.resize(v1.size());
    Eigen::VectorXf::Map(&v2[0], v1.size()) = v1;
    plt::named_plot("X Axis", t, v2);
    
    v1 = gyro_allan_variance.col(1);    
    v2.resize(v1.size());
    Eigen::VectorXf::Map(&v2[0], v1.size()) = v1;
    plt::named_plot("Y Axis", t, v2);
    
    v1 = gyro_allan_variance.col(2);   
    v2.resize(v1.size());
    Eigen::VectorXf::Map(&v2[0], v1.size()) = v1;
    plt::named_plot("Z Axis", t, v2);

    plt::ylabel("Allan Variance [rad^2/s^2]");
    plt::xlabel("Time [s]");
    plt::title("Gyroscope Allan Variance");
    plt::legend();
    plt::show();
    return 0;
}