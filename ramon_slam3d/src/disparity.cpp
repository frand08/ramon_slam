#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include <time.h>
#include <iostream>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <stereo_msgs/DisparityImage.h>
#define MISSING_Z 10000

#include <boost/version.hpp>

inline bool isValidPoint(const cv::Vec3f& pt)
{
  // Check both for disparities explicitly marked as invalid (where OpenCV maps pt.z to MISSING_Z)
  // and zero disparities (point mapped to infinity).
  return pt[2] != MISSING_Z && !std::isinf(pt[2]);
}

int main (int argc, char **argv)
{
    ros::init (argc, argv, "stereo_disparity");
    ros::NodeHandle nh("~");
    ros::Publisher pub_points2;
    // In BGR by default
    cv::VideoCapture left("/dev/video4"), right("/dev/video2");
    double width, height;
    double fps;
    pub_points2 = nh.advertise<sensor_msgs::PointCloud2>("points2",  1);
    // K: intrinsic camera matrix for the raw (distorted) images

    // D: distortion parameters/coeffs

    // P: Projection/camera matrix
    // XX: Tx for right camera, 0 for left camera
    // [f_rect, alpha, c_x ,      [1, 0, 0, XX ,
    //    0   ,  f_y , c_y ,  *    0, 1, 0,  0 ,
    //    0   ,   0  ,  1 ]        0, 0, 1,  0];
    // By convention, this matrix specifies the intrinsic (camera) matrix
    //  of the processed (rectified) image. That is, the left 3x3 portion
    //  is the normal camera intrinsic matrix for the rectified image.
    // It projects 3D points in the camera coordinate frame to 2D pixel
    //  coordinates using the focal lengths (fx', fy') and principal point
    //  (cx', cy') - these may differ from the values in K.
    // For monocular cameras, Tx = Ty = 0. Normally, monocular cameras will
    //  also have R = the identity and P[1:3,1:3] = K.
    // For a stereo pair, the fourth column [Tx Ty 0]' is related to the
    //  position of the optical center of the second camera in the first
    //  camera's frame. We assume Tz = 0 so both cameras are in the same
    //  stereo image plane. The first camera always has Tx = Ty = 0. For
    //  the right (second) camera of a horizontal stereo pair, Ty = 0 and
    //  Tx = -fx' * B, where B is the baseline between the cameras.
    // Given a 3D point [X Y Z]', the projection (x, y) of the point onto
    //  the rectified image is given by:
    //  [u v w]' = P * [X Y Z 1]'
    //         x = u / w
    //         y = v / w
    // This holds for both images of a stereo pair.


    // R: Rectification matrix, for stereo cameras only.
    // A rotation matrix aligning the camera coordinate system to the ideal
    //  stereo image plane so that epipolar lines in both stereo images are
    //  parallel.

    /* FIXME: Read data from yaml files */
    // From 20200327_320x240
    // Right camera
    cv::Mat K_right = (cv::Mat_<double>(3,3) << 435.736  ,    0.     ,  163.48678,
                                                  0.     ,  436.74973,  117.12178,
                                                  0.     ,    0.     ,    1.     );

    cv::Mat D_right = (cv::Mat_<double>(1,5) << -0.139272, 0.228863, 0.000443, 0.002089, 0.000000);

    cv::Mat P_right = (cv::Mat_<double>(3,4) << 453.19158,    0.     ,  159.7411 ,  -27.55073,
                                                  0.     ,  453.19158,  110.3157 ,    0.     ,
                                                  0.     ,    0.     ,    1.     ,    0.     );

    cv::Mat R_right = (cv::Mat_<double>(3,3) << 0.99997044, -0.00746816,  0.00182902,
                                                0.00746328,  0.99996861,  0.00266238,
                                               -0.00184884, -0.00264865,  0.99999478);

    // Left camera
    cv::Mat K_left = (cv::Mat_<double>(3,3) << 431.44958,    0.     ,  151.48809,
                                                 0.     ,  432.60045,  103.73964,
                                                 0.     ,    0.     ,    1.     );

    cv::Mat D_left = (cv::Mat_<double>(1,5) << -0.088844, -0.120207, 0.001345, -0.002007, 0.000000);

    cv::Mat P_left = (cv::Mat_<double>(3,4) << 453.19158,    0.     ,  159.7411 ,    0.     ,
                                                 0.     ,  453.19158,  110.3157 ,    0.     ,
                                                 0.     ,    0.     ,    1.     ,    0.     );

    cv::Mat R_left = (cv::Mat_<double>(3,3) <<   0.99989833, -0.0080206 , -0.01178974,
                                                 0.00798926,  0.99996443, -0.0027027 ,
                                                 0.011811  ,  0.00260823,  0.99992685);

    double c_x = P_left.at<double>(0,2);
    double c_y = P_left.at<double>(1,2);
    double f = P_left.at<double>(1,1);
    double cr_x = P_right.at<double>(0,2);

    // Q: reprojection matrix, defined as
    // [1, 0,   0  ,     -c_x       ,
    //  0, 1,   0  ,     -c_y       ,
    //  0, 0,   0  ,       f        ,
    //  0, 0, -1/Tx, (c_x-c'_x)/T_x];
    // Here the parameters are from the left image except for c'_x

    float T_x = P_right.at<double>(0,3) / P_right.at<double>(0,0);

    double Q03, Q13, Q23, Q32, Q33;
    Q03 = -c_x;
    Q13 = -c_y;
    Q23 = f;
    Q32 = -1/T_x;
    Q33 = (c_x - cr_x)/T_x;
    double data[16] = {1, 0,  0 , Q03,
                       0, 1,  0 , Q13,
                       0, 0,  0 , Q23,
                       0, 0, Q32, Q33}; 
    cv::Mat Q(4, 4, CV_64F, data);

    width = 320;
    height = 240;
    fps = 30.0;

    cv::Size image_size;
    image_size.height = height;
    image_size.width = width;

    // Resize images
    left.set(cv::CAP_PROP_FRAME_WIDTH, width);
    left.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    right.set(cv::CAP_PROP_FRAME_WIDTH, width);
    right.set(cv::CAP_PROP_FRAME_HEIGHT, height);

    // Set fps
    left.set(cv::CAP_PROP_FPS, fps);
    right.set(cv::CAP_PROP_FPS, fps);

    // Use MJPEG to avoid overloading the USB 2.0 bus
    left.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    right.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

    // Grab one to get frame info
    cv::Mat imgL, imgR;
    if(!(left.read(imgL) && right.read(imgR)))
    {
        ROS_ERROR("Unable to grab some CAM");
        left.release();
        right.release();
        return -1;
    }

    if(imgL.size() != imgR.size() || imgL.type() != imgR.type())
    {
        ROS_ERROR("The cameras use different framesize or type");
        left.release();
        right.release();
        return -1;
    }
    
    // ENSURE RIGHT TYPE AND SIZE
    image_size = imgL.size();
    // this will hold images from both cams 
    cv::Mat frame(cv::Size(2 * image_size.width, image_size.height), imgL.type());
    // define ROI to compose mosaic
    cv::Mat frameL(frame(cv::Rect(0, 0, image_size.width, image_size.height)));
    cv::Mat frameR(frame(cv::Rect(image_size.width, 0, image_size.width, image_size.height)));

    // Steps:
    // 1) Image raw
    // 2) Undistort
    // 3) Rectify
    // 4) Crop

    // Build the undistort map which we will use for all subsequent frames.
    // p. 762 OpenCV 3 book
    // For R, use Rl and Rr from cv::stereoRectify(); for cameraMatrix, 
    // use cameraMatrix1 or cameraMatrix2. For newCameraMatrix we could use 
    // the first three columns of the 3 × 4 Pl or Rr from cv::stereoRectify(), but as a
    // convenience the function allows us to pass Pl or Pr directly and it will read 
    // newCamera Matrix from them
    /* FIXME: CV_16SC2 or CV_32C1? */
    cv::Mat map1_right, map2_right;
    cv::initUndistortRectifyMap(K_right, D_right, R_right,
    P_right, image_size, CV_16SC2, map1_right, map2_right
    );

    cv::Mat map1_left, map2_left;
    cv::initUndistortRectifyMap(K_left, D_left, R_left,
    P_left, image_size, CV_16SC2, map1_left, map2_left
    );

    // Disparity image algorithm initialization
    // Elegir buenos parametros!!!!
    // int minDisparity = -64;
    // int numDisparities = 128;
    // int blockSize = 11;
    // int P1 = 100;
    // int P2 = 1000;
    // int disp12MaxDiff = 32;
    // int preFilterCap = 0;
    // int uniquenessRatio = 15;
    // int speckleWindowSize = 1000;
    // int speckleRange = 16;
    // int mode = cv::StereoSGBM::MODE_HH;
    int minDisparity = 0;
    int numDisparities = 128;
    int blockSize = 5;
    int P1 = 0;
    int P2 = 0;
    int disp12MaxDiff = 0;
    int preFilterCap = 31;
    int uniquenessRatio = 15;
    int speckleWindowSize = 500;
    int speckleRange = 4;
    int mode = cv::StereoSGBM::MODE_HH;
    cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create(minDisparity, numDisparities, blockSize, 
                                                            P1, P2, 
                                                            disp12MaxDiff, preFilterCap, uniquenessRatio, 
                                                            speckleWindowSize, speckleRange, 
                                                            mode);
    

    ros::Rate r(1000.0);
    while(nh.ok())
    {
        // Grab both frames first, then retrieve to minimize latency between cameras
        if(!(left.grab() && right.grab()))
        {
            ROS_ERROR("Unable to grab from one or both cameras");
            break;
        }

        cv::Mat right_buff, left_buff;
        cv::Mat right_buff_rect, left_buff_rect;
        left.retrieve(left_buff);
        right.retrieve(right_buff);

        if(right_buff.empty() || left_buff.empty())
        {
            ROS_ERROR("Empty frame received from one or both cameras");
            break;
        }

        // 2) 3) Undistort and Rectify images
        cv::remap(left_buff, left_buff_rect, map1_left, map2_left, cv::INTER_LINEAR);
        cv::remap(right_buff, right_buff_rect, map1_right, map2_right, cv::INTER_LINEAR);

        cv::Mat disparity, vdisparity;

        stereo->compute(left_buff_rect, right_buff_rect, disparity);
        cv::normalize(disparity, vdisparity, 0, 256, cv::NORM_MINMAX, CV_8U);

        if(left_buff_rect.size() != vdisparity.size() || right_buff_rect.size() != vdisparity.size())
        {
            ROS_ERROR("Rectified images and disparity image have different sizes");
            return -1;
        }
        // Get depth map from disparity
        cv::Mat_<cv::Vec3f> depth(vdisparity.size(), CV_32FC3);

        /* FIXME: Check if handleMissingValues is required */
        cv::reprojectImageTo3D(vdisparity, depth, Q);

        sensor_msgs::PointCloud2Ptr points_msg = boost::make_shared<sensor_msgs::PointCloud2>();
        points_msg->header.stamp = ros::Time::now();
        points_msg->header.frame_id = "map";
        points_msg->height = depth.rows;
        points_msg->width = depth.cols;
        points_msg->is_bigendian = false;
        points_msg->is_dense = false;

        // Used to conveniently set the fields structure, resize or clear a point cloud.
        sensor_msgs::PointCloud2Modifier pcd_modifier(*points_msg);
        // This call also resizes the data structure according to the given width, height and fields
        pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

        // Used to iterate the fields (like "x" for the x-coordinate, or "r" for the red color)
        sensor_msgs::PointCloud2Iterator<float> iter_x(*points_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*points_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*points_msg, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*points_msg, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*points_msg, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*points_msg, "b");

        float bad_point = std::numeric_limits<float>::quiet_NaN ();
        for (int v = 0; v < depth.rows; ++v)
        {

            for (int u = 0; u < depth.cols; ++u, ++iter_x, ++iter_y, ++iter_z)
            {
                if (isValidPoint(depth(v,u)))
                {
                    // x,y,z
                    *iter_x = depth(v, u)[0];
                    *iter_y = depth(v, u)[1];
                    *iter_z = depth(v, u)[2];
                }
                else
                {
                    *iter_x = *iter_y = *iter_z = bad_point;
                }
            }
        }

        // Fill in color (BGR8)
        const cv::Mat_<cv::Vec3b> color(left_buff_rect.rows, left_buff_rect.cols,
                                        (cv::Vec3b*) &left_buff.data[0],
                                        left_buff_rect.step);

        for (int v = 0; v < color.rows; ++v)
        {
            for (int u = 0; u < color.cols; ++u, ++iter_r, ++iter_g, ++iter_b)
            {
                const cv::Vec3b &bgr = color(v,u);
                *iter_r = bgr[2];
                *iter_g = bgr[1];
                *iter_b = bgr[0];
            }
        }
        
        left_buff_rect.copyTo(frameL);
        right_buff_rect.copyTo(frameR);
        cv::imshow("camera_frame", frame);
        cv::imshow("disparity", vdisparity);
        cv::imshow("depth", depth);

        pub_points2.publish(points_msg);
        cv::waitKey(1);
        ros::spinOnce();
        // r.sleep();
    }

    left.release();
    right.release();
    cv::destroyAllWindows();

    return 0;
}