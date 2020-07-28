#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/MagneticField.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

int main (int argc, char** argv)
{
    ros::init (argc, argv, "mag_calibration");    
    rosbag::Bag bag, bag2;
    bag.open("/home/fdominguez/Documents/bagfiles/mag_cal.bag", rosbag::bagmode::Read);
    bag2.open("/home/fdominguez/Documents/bagfiles/mag_cal_out.bag",rosbag::bagmode::Write);
    std::vector<float> x_data, y_data, z_data;
    std::vector<std::string> topics;
    topics.push_back(std::string("/mag_data"));

    geometry_msgs::Vector3 vec;
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    ros::Time::init();

    ROS_INFO("Entrando");
    foreach(rosbag::MessageInstance const m, view)
    {
        sensor_msgs::MagneticField::ConstPtr i = m.instantiate<sensor_msgs::MagneticField>();
        if (i != NULL)
        {
            vec.x = i->magnetic_field.x;
            vec.y = i->magnetic_field.y;
            vec.z = i->magnetic_field.z;
            bag2.write("mag_data_vec",ros::Time::now(), vec);
        }
    }

    bag.close();
    bag2.close();
    return 0;
}