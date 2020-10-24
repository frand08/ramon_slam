// https://stackoverflow.com/questions/30559556/robust-registration-of-two-point-clouds

#include <ros/ros.h>

#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/search/kdtree.h>
// #include <pcl/keypoints/harris_3d.h>
// #include <pcl/keypoints/susan.h>
#include <pcl/filters/uniform_sampling.h>

#include <pcl/keypoints/sift_keypoint.h>

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/transformation_estimation_svd.h>

// Types
typedef pcl::PointXYZRGB PointInT;
typedef pcl::PointCloud<PointInT> PointCloudInT;
typedef pcl::PointXYZRGB PointOutT;
typedef pcl::PointCloud<PointOutT> PointCloudOutT;
typedef pcl::PointNormal NormalT;
typedef pcl::PointCloud<NormalT> PointCloudNormalT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::PointCloud<FeatureT> PointCloudFeatureT;

//////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4f ICP(const PointCloudInT::Ptr &cloud_prev,
                    const PointCloudInT::Ptr &cloud_reg)
{
  pcl::IterativeClosestPoint<PointInT, PointInT> icp;

  // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
  icp.setMaxCorrespondenceDistance (0.05);
  // Set the maximum number of iterations (criterion 1)
  icp.setMaximumIterations (100);
  // Set the transformation epsilon (criterion 2)
  icp.setTransformationEpsilon (1e-8);
  // Set the euclidean distance difference epsilon (criterion 3)
  icp.setEuclideanFitnessEpsilon (1);
  // icp.setRANSACOutlierRejectionThreshold (1.5);

  // Set input cloud and previous pointcloud
  icp.setInputSource(cloud_prev);
  // icp.setInputTarget(scene_transformed);
  icp.setInputTarget(cloud_reg);

  // Perform ICP
  pcl::PointCloud<PointInT>::Ptr Final(new pcl::PointCloud<PointInT>);
  icp.align(*Final);
  return (icp.getFinalTransformation());
}
//////////////////////////////////////////////////////////////////////////////////////
void
estimateFPFH(const PointCloudInT::Ptr &src,
             const PointCloudNormalT::Ptr &normals_src,
             const PointCloudInT::Ptr &keypoints_src,
             float radius,
             PointCloudFeatureT &fpfhs_src)
{
  pcl::FPFHEstimationOMP<PointInT, NormalT, FeatureT> fpfh_est;
  fpfh_est.setInputCloud(keypoints_src);
  fpfh_est.setInputNormals(normals_src);
  // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
  fpfh_est.setRadiusSearch(radius); // 1m
  fpfh_est.setSearchSurface(src);
  fpfh_est.compute(fpfhs_src);
}

//////////////////////////////////////////////////////////////////////////////////////
void
estimateKeypoints(const PointCloudInT::Ptr &src,
                  float radius,
                  PointCloudInT &keypoints_src)
{
  // Get an uniform grid of keypoints
  pcl::UniformSampling<PointInT> uniform;
  uniform.setRadiusSearch(radius); //1m

  uniform.setInputCloud(src);
  uniform.filter(keypoints_src);
}
//////////////////////////////////////////////////////////////////////////////////////
void
estimateSIFTKeypoints(const PointCloudInT::Ptr &src,
                      const float min_scale, const int n_octaves,
                      const int n_scales_per_octave,
                      const float min_contrast,
                      PointCloudInT &keypoints_src)
{
  pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> sift;
  pcl::PointCloud<pcl::PointWithScale> result;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree =
          boost::make_shared<pcl::search::KdTree<pcl::PointXYZRGB>>();

  sift.setSearchMethod(tree);
  sift.setScales(min_scale, n_octaves, n_scales_per_octave);
  sift.setMinimumContrast(min_contrast);
  sift.setInputCloud(src);
  sift.compute(result);
  copyPointCloud(result, keypoints_src);
}

////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4f RANSAC(const PointCloudInT::Ptr &cloud_prev,
                       const PointCloudFeatureT::Ptr &cloud_prev_features,
                       const PointCloudInT::Ptr &cloud_reg,
                       const PointCloudFeatureT::Ptr &cloud_reg_features,
                       float leaf)
{
  PointCloudInT::Ptr cloud_reg_aligned(new PointCloudInT);

  // Perform alignment
  pcl::console::print_highlight ("Starting alignment...\n");
  pcl::SampleConsensusPrerejective<PointInT,PointInT,FeatureT> align;
  align.setInputSource (cloud_prev);
  align.setSourceFeatures (cloud_prev_features);
  align.setInputTarget (cloud_reg);
  align.setTargetFeatures (cloud_reg_features);
  align.setMaximumIterations (5000); // Number of RANSAC iterations
  align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
  align.setCorrespondenceRandomness (5); // Number of nearest features to use
  align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
  align.setMaxCorrespondenceDistance (2.5f * leaf); // Inlier threshold
  align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
  align.align (*cloud_reg_aligned);

  return(align.getFinalTransformation());
}

//////////////////////////////////////////////////////////////////////////////////////
void
estimateNormals(const PointCloudInT::Ptr &src,
                float radius,
                PointCloudNormalT &normals_src)
{
  // Create an empty kdtree representation, and pass it to the normal estimation
  // object.
  // Its content will be filled inside the object, based on the given input
  // dataset (as no other search surface is given).
  pcl::search::KdTree<PointInT>::Ptr tree =
          boost::make_shared<pcl::search::KdTree<PointInT>>();
  pcl::NormalEstimation<PointInT, NormalT> normal_est;
  normal_est.setSearchMethod(tree);
  normal_est.setInputCloud(src);
  normal_est.setRadiusSearch(radius);  // 50cm
  normal_est.compute(normals_src);
}
//////////////////////////////////////////////////////////////////////////////////////
void
estimateNormalsOMP(const PointCloudInT::Ptr &src, 
                   float radius,
                   PointCloudNormalT &normals_src)
{
  pcl::NormalEstimationOMP<PointInT, NormalT> ne;
  // Create an empty kdtree representation, and pass it to the normal estimation
  // object.
  // Its content will be filled inside the object, based on the given input
  // dataset (as no other search surface is given).
  pcl::search::KdTree<PointInT>::Ptr tree =
          boost::make_shared<pcl::search::KdTree<PointInT>>();

  ne.setNumberOfThreads(8);
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(radius);
  ne.setInputCloud(src);
  ne.compute(normals_src);
  Eigen::Vector4f centroid;
  compute3DCentroid(*src, centroid);
  ne.setViewPoint(centroid[0], centroid[1], centroid[2]);
  for (size_t i = 0; i < normals_src.size(); ++i) {
      normals_src.points[i].normal_x *= -1;
      normals_src.points[i].normal_y *= -1;
      normals_src.points[i].normal_z *= -1;
  }
}

////////////////////////////////////////////////////////////////////////////////
void
findCorrespondences(const PointCloudFeatureT::Ptr &fpfhs_src,
                    const PointCloudFeatureT::Ptr &fpfhs_tgt,
                    pcl::Correspondences &all_correspondences)
{
  pcl::registration::CorrespondenceEstimation<FeatureT, FeatureT> est;
  est.setInputSource(fpfhs_src);
  est.setInputTarget(fpfhs_tgt);
  est.determineReciprocalCorrespondences(all_correspondences);
}

////////////////////////////////////////////////////////////////////////////////
void
rejectBadCorrespondences(const pcl::CorrespondencesPtr &all_correspondences,
                         const PointCloudInT::Ptr &keypoints_src,
                         const PointCloudInT::Ptr &keypoints_tgt,
                         float max_dist,
                         pcl::Correspondences &remaining_correspondences)
{
  pcl::registration::CorrespondenceRejectorMedianDistance rej;
  rej.setInputSource<PointInT>(keypoints_src);
  rej.setInputTarget<PointInT>(keypoints_tgt);
  // rej.setMaximumDistance (max_dist);    // 1m
  rej.setInputCorrespondences(all_correspondences);
  rej.getCorrespondences(remaining_correspondences);
}

//////////////////////////////////////////////////////////////////////////////////////
// Align a rigid object to a scene with clutter and occlusions
int
main (int argc, char **argv)
{
  ros::init(argc, argv, "pcl_test");

  // Node handle
  ros::NodeHandle nh("~");

  std::string cloud_reg_str;
  std::string cloud_prev_str;

  // Point clouds
  PointCloudInT::Ptr cloud1(new PointCloudInT);
  PointCloudInT::Ptr cloud20(new PointCloudInT);
  PointCloudInT::Ptr cloud40(new PointCloudInT);
  PointCloudInT::Ptr cloud65(new PointCloudInT);
  PointCloudInT::Ptr cloud80(new PointCloudInT);
  PointCloudInT::Ptr map(new PointCloudInT);
  std::vector < PointCloudInT::Ptr, Eigen::aligned_allocator <PointCloudInT::Ptr> > clouds;
  std::vector < PointCloudInT::Ptr, Eigen::aligned_allocator <PointCloudInT::Ptr> > clouds_keypoints;
  std::vector < PointCloudNormalT::Ptr, Eigen::aligned_allocator <PointCloudNormalT::Ptr> > clouds_normal;
  std::vector < PointCloudFeatureT::Ptr, Eigen::aligned_allocator <PointCloudFeatureT::Ptr> > clouds_features;

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
  if (pcl::io::loadPCDFile<PointInT>(cloud_prev_str, *cloud1) < 0 ||
      pcl::io::loadPCDFile<PointInT>(cloud_reg_str, *cloud20) < 0)  
  {
    pcl::console::print_error("Error loading file!\n");
    return (1);
  }
  clouds.push_back(cloud1);

  Eigen::Matrix4f transform;
  Eigen::Vector3f rotationAxis;
  Eigen::Matrix4f tr;
  rotationAxis << 0, 
                  0, 
                  1;
  float angle = M_PI * 0.2;
  Eigen::Isometry3f pose(Eigen::Isometry3f(Eigen::Translation3f(
                        0.0,
                        0.0, 
                        0.2)) * 
                        Eigen::Isometry3f(Eigen::AngleAxisf(angle, rotationAxis))
                        );
  tr = pose.matrix();
  // pcl::transformPointCloud(*cloud1, *cloud20, tr);
  clouds.push_back(cloud20);
  *map = *cloud1 + *cloud20;

  // pcl::console::print_highlight("Real transform\n");
  // pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", tr (0,0), tr (0,1), tr (0,2));
  // pcl::console::print_info("R = | %6.3f %6.3f %6.3f | \n", tr (1,0), tr (1,1), tr (1,2));
  // pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", tr (2,0), tr (2,1), tr (2,2));
  // pcl::console::print_info("\n");
  // pcl::console::print_info("t = < %0.3f, %0.3f, %0.3f >\n", tr (0,3), tr (1,3), tr (2,3));
  // pcl::console::print_info("\n");


  pcl::console::print_highlight("Downsampling...\n");
  pcl::VoxelGrid<PointInT> grid;
  const float leaf = 0.1f;
  grid.setLeafSize(leaf, leaf, leaf);
  for(int i = 0; i < clouds.size(); i++)
  {
    grid.setInputCloud(clouds[i]);
    grid.filter(*clouds[i]);    
  }
  pcl::console::print_info("leaf size = %f\n", leaf);


  pcl::console::print_highlight("Estimating keypoints (SIFT)...\n");
  // Get an uniform grid of keypoints
  for(int i = 0; i < clouds.size(); i++)
  {
    PointCloudInT::Ptr keypoints_src(new PointCloudInT);
    estimateSIFTKeypoints(clouds[i], 0.005, 10, 8, 1.5, *keypoints_src);
// const float min_scale = 0.0005; 
// const int nr_octaves = 4; 
// const int nr_scales_per_octave = 5; 
// const float min_contrast = 1; 
    // estimateSIFTKeypoints(clouds[i], 0.0005, 4, 5, 1, *keypoints_src);
    // estimateKeypoints(clouds[i], 1, *keypoints_src);
    clouds_keypoints.push_back(keypoints_src);
  }


  pcl::console::print_highlight("Estimating normals (OMP)...\n");
  for(int i = 0; i < clouds.size(); i++)
  {
    PointCloudNormalT::Ptr normals_src(new PointCloudNormalT);
    // estimateNormals(clouds[i], .5, *normals_src);
    // estimateNormalsOMP(clouds[i], 0.01, *normals_src);
    estimateNormalsOMP(clouds[i], 0.5, *normals_src);
    clouds_normal.push_back(normals_src);
  }

  pcl::console::print_highlight("Estimating FPFH features (OMP)...\n");
  // Compute FPFH features at each keypoint
  for(int i = 0; i < clouds.size(); i++)
  {
    PointCloudFeatureT::Ptr fpfhs_src(new PointCloudFeatureT);
    estimateFPFH(clouds[i], clouds_normal[i], clouds_keypoints[i], 1.0, *fpfhs_src);
    clouds_features.push_back(fpfhs_src);
  }

  pcl::console::print_highlight("Finding correspondences between keypoints...\n");
  // Find correspondences between keypoints in FPFH space
  pcl::CorrespondencesPtr all_correspondences (new pcl::Correspondences), 
                          good_correspondences (new pcl::Correspondences);
  findCorrespondences(clouds_features[0], clouds_features[1], *all_correspondences);

  pcl::console::print_highlight("Rejecting correspondences based on their XYZ distance (Median)...\n");
  // Reject correspondences based on their XYZ distance
  rejectBadCorrespondences(all_correspondences, clouds_keypoints[0], clouds_keypoints[1], 1, *good_correspondences);

  // pcl::console::print_highlight("RANSAC...\n");
  // Eigen::Matrix4f r_tr = RANSAC(clouds_keypoints[0], clouds_features[0], clouds_keypoints[1], clouds_features[1], leaf);

  // pcl::console::print_highlight("RANSAC estimated transform\n");
  // pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", r_tr (0,0), r_tr (0,1), r_tr (0,2));
  // pcl::console::print_info("R = | %6.3f %6.3f %6.3f | \n", r_tr (1,0), r_tr (1,1), r_tr (1,2));
  // pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", r_tr (2,0), r_tr (2,1), r_tr (2,2));
  // pcl::console::print_info("\n");
  // pcl::console::print_info("t = < %0.3f, %0.3f, %0.3f >\n", r_tr (0,3), r_tr (1,3), r_tr (2,3));
  // pcl::console::print_info("\n");

  // pcl::transformPointCloud(*clouds_features[0], *clouds_features[0], r_tr);
  // pcl::transformPointCloud(*clouds_keypoints[0], *clouds_keypoints[0], r_tr);

  // pcl::transformPointCloud(*clouds[0], *clouds[0], r_tr);
  // pcl::transformPointCloud(*clouds[1], *clouds[1], r_tr);

  pcl::console::print_highlight("Estimating transform...\n");
  // Obtain the best transformation between the two sets of keypoints given the remaining correspondences
  pcl::registration::TransformationEstimationSVD<PointInT, PointInT> trans_est;
  trans_est.estimateRigidTransformation(*clouds_keypoints[0], *clouds_keypoints[1], *good_correspondences, transform);
  // transform = ICP(clouds_keypoints[0], clouds_keypoints[1]);
  // transform = ICP(clouds[0], clouds[1]);

  // transform = r_tr * transform;
  pcl::console::print_highlight("ICP estimated transform\n");
  pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transform (0,0), transform (0,1), transform (0,2));
  pcl::console::print_info("R = | %6.3f %6.3f %6.3f | \n", transform (1,0), transform (1,1), transform (1,2));
  pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transform (2,0), transform (2,1), transform (2,2));
  pcl::console::print_info("\n");
  pcl::console::print_info("t = < %0.3f, %0.3f, %0.3f >\n", transform (0,3), transform (1,3), transform (2,3));
  pcl::console::print_info("\n");
  
  ros::spinOnce();
  
  return (0);
}