#include "imu_calibration.h"


int main (int argc, char** argv)
{
    ros::init(argc, argv, "imu");
    ramon_calibration::IMUCalibration cal;
    cal.start();
    ROS_INFO_STREAM("IMU calibration result " << cal.calibrate());

    return 0;
}