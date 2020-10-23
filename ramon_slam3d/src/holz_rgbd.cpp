#include <ros/ros.h>

#include "map3d_utils.h"
#include <pcl/visualization/pcl_visualizer.h>

using namespace ramon_slam3d;

typedef pcl::PointXYZRGB PointSourceT;
typedef pcl::PointCloud<PointSourceT> PointCloudSourceT;
typedef pcl::PointXYZRGBNormal PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

void viewCloud(PointCloudSourceT::Ptr &cloud1)
{
  pcl::visualization::PCLVisualizer viewer ("3D cloud");
  
  viewer.addPointCloud(cloud1);
  while(!viewer.wasStopped())
    viewer.spinOnce ();
  viewer.close();
}

int
main (int argc, char **argv)
{
  ros::init(argc, argv, "holz_rgbd");

  // Node handle
  ros::NodeHandle nh("~");

  std::string cloud_reg_str;
  std::string cloud_prev_str;
  
  // Point clouds
  PointCloudSourceT::Ptr cloud1(new PointCloudSourceT);
  PointCloudSourceT::Ptr cloud1_filtered(new PointCloudSourceT);
  
  PointCloudSourceT::Ptr cloud20(new PointCloudSourceT);
  PointCloudSourceT::Ptr cloud20_filtered(new PointCloudSourceT);


  Eigen::Matrix4d transform, transform_init, transform_init_inv;

  Map3DUtils<PointSourceT, PointT> map3d;

  pcl::CorrespondencesPtr all_correspondences (new pcl::Correspondences), 
                          good_correspondences (new pcl::Correspondences);

  ros::WallTime start, end;

  if (!nh.getParam("cloud_reg", cloud_reg_str))
  {
    ROS_ERROR("Input cloud error");
    return -1;
  }

  if (!nh.getParam("cloud_prev", cloud_prev_str))
  {
    ROS_ERROR("Input cloud error");
    return -1;
  }

  // Load object and scene
  pcl::console::print_highlight("Loading point clouds...\n");
  if (pcl::io::loadPCDFile<PointSourceT>(cloud_prev_str, *cloud1) < 0 ||
      pcl::io::loadPCDFile<PointSourceT>(cloud_reg_str, *cloud20) < 0)  
  {
    pcl::console::print_error("Error loading file!\n");
    return (1);
  }

  start = ros::WallTime::now();

  transform_init << 0.9900333 , -0.0993347,  0.0998334, 0.1,
                    0.1092516 ,  0.9890383, -0.0993347,  0 ,
                    -0.0888717,  0.1092516,  0.9900333,  0 ,
                          0   ,      0    ,       0   ,  1 ;
  map3d.filterAndUpsample(cloud20, *cloud20_filtered);
  map3d.filterAndUpsample(cloud1, *cloud1_filtered);
  // pcl::transformPointCloud(*cloud1_filtered, *cloud20_filtered, transform_init);

  map3d.registrationPipelineHolz(cloud20_filtered, cloud1_filtered, transform);
  end = ros::WallTime::now();
  double execution_time = (end - start).toNSec() * 1e-6;
  ROS_INFO_STREAM("Exectution time (ms): " << execution_time);
  
  pcl::console::print_highlight("Real transform\n");
  pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transform_init (0,0), transform_init (0,1), transform_init (0,2));
  pcl::console::print_info("R = | %6.3f %6.3f %6.3f | \n", transform_init (1,0), transform_init (1,1), transform_init (1,2));
  pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transform_init (2,0), transform_init (2,1), transform_init (2,2));
  pcl::console::print_info("\n");
  pcl::console::print_info("t = < %0.3f, %0.3f, %0.3f >\n", transform_init (0,3), transform_init (1,3), transform_init (2,3));
  pcl::console::print_info("\n");

  transform_init_inv << transform_init.block(0,0,3,3).inverse(), -transform_init.block(0,0,3,3).inverse()*transform_init.block(0,3,3,1),
                        0, 0, 0, 1;  
  pcl::console::print_highlight("Inverse real transform\n");
  pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transform_init_inv (0,0), transform_init_inv (0,1), transform_init_inv (0,2));
  pcl::console::print_info("R = | %6.3f %6.3f %6.3f | \n", transform_init_inv (1,0), transform_init_inv (1,1), transform_init_inv (1,2));
  pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transform_init_inv (2,0), transform_init_inv (2,1), transform_init_inv (2,2));
  pcl::console::print_info("\n");
  pcl::console::print_info("t = < %0.3f, %0.3f, %0.3f >\n", transform_init_inv (0,3), transform_init_inv (1,3), transform_init_inv (2,3));
  pcl::console::print_info("\n");

  pcl::console::print_highlight("Estimated transform\n");
  pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transform (0,0), transform (0,1), transform (0,2));
  pcl::console::print_info("R = | %6.3f %6.3f %6.3f | \n", transform (1,0), transform (1,1), transform (1,2));
  pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transform (2,0), transform (2,1), transform (2,2));
  pcl::console::print_info("\n");
  pcl::console::print_info("t = < %0.3f, %0.3f, %0.3f >\n", transform (0,3), transform (1,3), transform (2,3));
  pcl::console::print_info("\n");

  ros::spinOnce();

  return (0);
}