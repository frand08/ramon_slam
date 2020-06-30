#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <ramon_calibration/ImuWithMag.h>
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

    ros::NodeHandle nh("~");

    std::string bag_file;
    std::string topic_name;
    float data_rate;

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

    bag.open(bag_file, rosbag::bagmode::Read);

    std::vector<std::string> topics;
    // topics.push_back(std::string("/imu_data"));
    topics.push_back(topic_name);
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    ros::Time::init();
    int j = 0;

    ROS_INFO_STREAM("Reading bag file " << bag_file << "...");    
    gyro_values.resize(bag.getSize(),3);
    foreach(rosbag::MessageInstance const mm, view)
    {
        /* TODO: poner condicional para ver de que tipo de msg se trata */
        // sensor_msgs::Imu::ConstPtr i = mm.instantiate<sensor_msgs::Imu>();
        ramon_calibration::ImuWithMag::ConstPtr i = mm.instantiate<ramon_calibration::ImuWithMag>();
        if (i != NULL)
        {
            gyro_values(j,0) = i->angular_velocity.x;
            gyro_values(j,1) = i->angular_velocity.y;
            gyro_values(j,2) = i->angular_velocity.z;
            j++;
        }
    }
    gyro_values.conservativeResize(j, 3);
    ROS_INFO("Done.");
    ROS_INFO("Performing the Allan Variance...");
    allan_variance(gyro_values, gyro_allan_variance);
    ROS_INFO("Done.");
    ROS_INFO("Allan Variance results:");
    // Plot the outputs
    Eigen::VectorXf v1 = gyro_allan_variance.col(0);
    std::vector<float> v2, t(gyro_allan_variance.rows());
    std::iota(t.begin(), t.end(), 0);

    std::transform(t.begin(), t.end(), t.begin(), [&data_rate](auto& c){return c*data_rate;});
    
    v2.resize(v1.size());
    Eigen::VectorXf::Map(&v2[0], v1.size()) = v1;
    plt::named_plot("Eje x", t, v2);
    
    v1 = gyro_allan_variance.col(1);    
    v2.resize(v1.size());
    Eigen::VectorXf::Map(&v2[0], v1.size()) = v1;
    plt::named_plot("Eje y", t, v2);
    
    v1 = gyro_allan_variance.col(2);   
    v2.resize(v1.size());
    Eigen::VectorXf::Map(&v2[0], v1.size()) = v1;
    plt::named_plot("Eje z", t, v2);

    plt::ylabel("Varianza de Allan [rad^2/s^2]");
    plt::xlabel("Tiempo [s]");
    // plt::title("Varianza de Allan de los datos del giroscopio");
    plt::legend();
    // plt::ion();
    plt::show();

    ROS_INFO("Allan Variance calculation finished. Press any key to exit");
    getchar();
    plt::close();

    return 0;
}