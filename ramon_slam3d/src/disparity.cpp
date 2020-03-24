#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include <time.h>
#include <iostream>
#include <ros/console.h>

using namespace cv;

int main (int argc, char **argv)
{
    ros::init (argc, argv, "stereo_disparity");
    ros::NodeHandle nh("~");
    cv::VideoCapture left("/dev/video4"), right("/dev/video2");
    double width, height;
    double fps;


    // P: Projection matrix
    // XX: Tx for right camera, 0 for left camera
    // [f_rect, alpha, c_x ,      [1, 0, 0, XX ,
    //    0   ,  f_y , c_y ,  *    0, 1, 0,  0 ,
    //    0   ,   0  ,  1 ]        0, 0, 1,  0];

    /* FIXME: Read data from yaml files */
    // From 20200318_320x240
    // Right camera
    cv::Mat K_right = (cv::Mat_<double>(3,3) << 442.95757,    0.     ,  165.6904 ,
                                                  0.     ,  444.10807,   98.11422,
                                                  0.     ,    0.     ,    1.     );

    cv::Mat P_right = (cv::Mat_<double>(3,4) << 502.87734,    0.     ,  208.39025,  -31.41567,
                                                  0.     ,  502.87734,   88.63361,    0.     ,
                                                  0.     ,    0.     ,    1.     ,    0.     );

    cv::Mat R_right = (cv::Mat_<double>(3,3) << 0.9964599 , -0.03414671, -0.07682227,
                                                0.03438375,  0.99940715,  0.00176472,
                                                0.07671646, -0.00439991,  0.99704324);

    cv::Mat D_right = (cv::Mat_<double>(1,5) << -0.101437, -0.016751, -0.010302, 0.003702, 0.000000);

    // Left camera
    cv::Mat K_left = (cv::Mat_<double>(3,3) << 432.60559,    0.     ,  167.65885,
                                                 0.     ,  433.84268,   81.72285,
                                                 0.     ,    0.     ,    1.     );

    cv::Mat P_left = (cv::Mat_<double>(3,4) << 502.87734,    0.     ,  208.39025,    0.     ,
                                                 0.     ,  502.87734,   88.63361,    0.     ,
                                                 0.     ,    0.     ,    1.     ,    0.     );

    cv::Mat R_left = (cv::Mat_<double>(3,3) <<  0.99559484, -0.03553375, -0.08676556,
                                                0.0352656 ,  0.99936729, -0.00462175,
                                                0.0868749 ,  0.00154155,  0.99621804);

    cv::Mat D_left = (cv::Mat_<double>(1,5) << -0.093483, -0.009592, -0.009334, 0.000425, 0.000000);

    width = 320;
    height = 240;
    fps = 30.0;

    // Resize images
    left.set(cv::CAP_PROP_FRAME_WIDTH, width);
    left.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    right.set(cv::CAP_PROP_FRAME_WIDTH, width);
    right.set(cv::CAP_PROP_FRAME_HEIGHT, height);

    // Set fps
    left.set(cv::CAP_PROP_FPS, fps);
    right.set(cv::CAP_PROP_FPS, fps);

    // Use MJPEG to avoid overloading the USB 2.0 bus
    // left.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    // right.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

    // Learning OpenCV 3 p. 710 aprox
    cv::Size image_size;
    image_size.height = height;
    image_size.width = width;


    // Steps:
    // 1) Image raw
    // 2) Undistort
    // 3) Rectify
    // 4) Crop

    // Build the undistort map which we will use for all subsequent frames. //
    cv::Mat map1_right, map2_right;
    cv::initUndistortRectifyMap( K_right, D_right, cv::Mat(),
    K_right, image_size, CV_16SC2, map1_right, map2_right
    );

    cv::Mat map1_left, map2_left;
    cv::initUndistortRectifyMap( K_left, D_left, cv::Mat(),
    K_left, image_size, CV_16SC2, map1_left, map2_left
    );

    ros::Rate r(1000.0);
    // Grab both frames first, then retrieve to minimize latency between cameras
    while(nh.ok())
    {
        ros::spinOnce();
        if(!(left.grab() && right.grab()))
        {
            ROS_ERROR("Unable to grab from one or both cameras");
            break;
        }

        cv::Mat right_buff, left_buff, right_buff_c1, left_buff_c1;
        right.retrieve(right_buff);
        left.retrieve(left_buff);

        if(right_buff.empty() || left_buff.empty())
        {
            ROS_ERROR("Empty frame received from one or both cameras");
            break;
        }


        // cv::remap(right_buff, right_buff, map1_right, map2_right, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
        // cv::remap(left_buff, left_buff, map1_left, map2_left, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());

        // 2) Undistort images
        cv::undistort(left_buff, left_buff_c1, K_left, D_left, K_left);
        cv::cvtColor(left_buff_c1, left_buff_c1, CV_BGR2GRAY);
        cv::undistort(right_buff, right_buff_c1, K_right, D_right, K_right);
        cv::cvtColor(right_buff_c1, right_buff_c1, CV_BGR2GRAY);

        // 3) Rectify
        // The rotation R is the identity because we don't want to rotate the image
        cv::Mat3f R;
        R << 1, 0, 0,
             0, 1, 0,
             0, 0, 1;
        
        cv::Vec3f T;
        T << 0,
             0,
             0;

        float T_x = P_right.at<float>(0,3) / P_right.at<float>(0,0);
        // Q: reprojection matrix, defined as
        // [1, 0,   0  ,     -c_x       ,
        //  0, 1,   0  ,     -c_y       ,
        //  0, 0,   0  ,       f        ,
        //  0, 0, -1/Tx, (c_x-c'_x)/T_x];
        // Here the parameters are from the left image except for c'_x
        cv::Mat4f Q;
        Q << 1, 0,   0   ,              -P_left.at<float>(0,2)                 ,
             0, 1,   0   ,              -P_left.at<float>(0,5)                 ,
             0, 0,   0   ,              -P_left.at<float>(0,0)                 ,
             0, 0, -1/T_x, (P_left.at<float>(0,2) - P_right.at<float>(0,2))/T_x;

        cv::stereoRectify(K_left, D_left, K_right, D_right, image_size, 
                          R, T, R_left, R_right, P_left, P_right, Q);




        cv::Mat disparity;
        cv::Ptr<StereoBM> sbm = cv::StereoBM::create();
        // sbm->setPreFilterSize(9);
        // sbm->setPreFilterCap(31);
        // sbm->setMinDisparity(0);
        // sbm->setNumDisparities(64);
        // sbm->setTextureThreshold(10);
        // sbm->setUniquenessRatio(0);
        // sbm->setSpeckleWindowSize(100);
        // sbm->setSpeckleRange(4);
        // sbm->setDisp12MaxDiff(1);
        sbm->setPreFilterSize(9);
        sbm->setPreFilterCap(61);
        sbm->setMinDisparity(-39);
        sbm->setNumDisparities(64);
        sbm->setTextureThreshold(500);
        sbm->setUniquenessRatio(0);
        sbm->setSpeckleWindowSize(100);
        sbm->setSpeckleRange(4);
        sbm->setDisp12MaxDiff(1);
        sbm->compute(left_buff_c1,right_buff_c1,disparity);
        cv::normalize(disparity, disparity, 0, 255, CV_MINMAX, CV_8U);

        cv::imshow("left", left_buff_c1);
        cv::imshow("right", right_buff_c1);
        cv::imshow("disparity",disparity);
        waitKey(1);
        
        r.sleep();
    }

    left.release();
    right.release();
    cv::destroyAllWindows();

    return 0;
}