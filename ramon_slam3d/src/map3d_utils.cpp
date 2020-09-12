/**
 * @file map3d_utils.cpp
 * @author Francisco A. Dominguez (dominguezfranciscoa@gmail.com)
 * @brief
 * @version 0.1
 * @date 2020-09-09
 *
 * @copyright Copyright (c) 2020
 *
 */

#include "map3d_utils.h"

using namespace ramon_slam3d;

/* Public Functions */

/**
 * @brief Construct a new Map3DUtils::Map3DUtils object
 *
 */
Map3DUtils::Map3DUtils()
{

}

/**
 * @brief Destroy the Map3DUtils::Map3DUtils object
 *
 */
Map3DUtils::~Map3DUtils()
{

}

/**
 * @brief Get best transform based on the Iterative Closest Point algorithm
 *
 */
int Map3DUtils::icp(const PointCloudT::Ptr &cloud_prev,
                    const PointCloudT::Ptr &cloud_reg,
                    Eigen::Matrix4f &transform,
                    float max_corresp_dist,
                    uint32_t max_iterations,
                    float transformation_epsilon,
                    float euclidean_fitness)
{
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  int ret = -1;

  // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
  icp.setMaxCorrespondenceDistance (max_corresp_dist);
  // Set the maximum number of iterations (criterion 1)
  icp.setMaximumIterations (max_iterations);
  // Set the transformation epsilon (criterion 2)
  icp.setTransformationEpsilon (transformation_epsilon);
  // Set the euclidean distance difference epsilon (criterion 3)
  icp.setEuclideanFitnessEpsilon (euclidean_fitness);
  // icp.setRANSACOutlierRejectionThreshold (1.5);

  // Set input cloud and previous pointcloud
  icp.setInputSource(cloud_prev);
  // icp.setInputTarget(scene_transformed);
  icp.setInputTarget(cloud_reg);

  // Perform ICP
  pcl::PointCloud<PointT>::Ptr Final(new pcl::PointCloud<PointT>);
  icp.align(*Final);
  if(icp.hasConverged())
  {
    transform = icp.getFinalTransformation();
    ret = 0;
  }
  return ret;
}

/**
 * @brief Get best transform based on the Iterative Closest Point algorithm
 *
 */
int Map3DUtils::estimateFPFH(const PointCloudT::Ptr &src,
                             const PointCloudNormalT::Ptr &normals_src,
                             const PointCloudT::Ptr &keypoints_src,
                             PointCloudFeatureT &fpfhs_src,
                             float radius)
{
  pcl::FPFHEstimationOMP<PointT, NormalT, FeatureT> fpfh_est;
  fpfh_est.setInputCloud(keypoints_src);
  fpfh_est.setInputNormals(normals_src);
  // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
  fpfh_est.setRadiusSearch(radius); // 1m
  fpfh_est.setSearchSurface(src);
  fpfh_est.compute(fpfhs_src);
  
  return 0;
}

/**
 * @brief Get best transform based on the Iterative Closest Point algorithm
 *
 */
int Map3DUtils::estimateKeypoints(const PointCloudT::Ptr &src,
                                  PointCloudT &keypoints_src,
                                  float radius)
{
  // Get an uniform grid of keypoints
  pcl::UniformSampling<PointT> uniform;
  uniform.setRadiusSearch(radius); //1m

  uniform.setInputCloud(src);
  uniform.filter(keypoints_src);

  return 0;
}

/**
 * @brief Get best transform based on the Iterative Closest Point algorithm
 *
 */
int Map3DUtils::estimateSIFTKeypoints(const PointCloudT::Ptr &src,
                                      PointCloudT &keypoints_src,
                                      const float min_scale, 
                                      const int n_octaves,
                                      const int n_scales_per_octave,
                                      const float min_contrast)
{
  pcl::SIFTKeypoint<PointT, PointScaleT> sift;
  PointCloudScaleT result;
  pcl::search::KdTree<PointT>::Ptr tree =
          boost::make_shared<pcl::search::KdTree<PointT>>();

  sift.setSearchMethod(tree);
  sift.setScales(min_scale, n_octaves, n_scales_per_octave);
  sift.setMinimumContrast(min_contrast);
  sift.setInputCloud(src);
  sift.compute(result);
  copyPointCloud(result, keypoints_src);

  return 0;
}

/**
 * @brief Get best transform based on the Iterative Closest Point algorithm
 *
 */
int Map3DUtils::ransac(const PointCloudT::Ptr &cloud_src,
                       const PointCloudFeatureT::Ptr &cloud_src_features,
                       const PointCloudT::Ptr &cloud_tgt,
                       const PointCloudFeatureT::Ptr &cloud_tgt_features,
                       Eigen::Matrix4f &transform,
                       float leaf)
{
  PointCloudT::Ptr cloud_tgt_aligned(new PointCloudT);
  int ret = -1;

  // Perform alignment
  pcl::console::print_highlight ("Starting alignment...\n");
  pcl::SampleConsensusPrerejective<PointT,PointT,FeatureT> align;
  align.setInputSource (cloud_src);
  align.setSourceFeatures (cloud_src_features);
  align.setInputTarget (cloud_tgt);
  align.setTargetFeatures (cloud_tgt_features);
  align.setMaximumIterations (5000); // Number of RANSAC iterations
  align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
  align.setCorrespondenceRandomness (5); // Number of nearest features to use
  align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
  align.setMaxCorrespondenceDistance (2.5f * leaf); // Inlier threshold
  align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
  align.align (*cloud_tgt_aligned);

  if(align.hasConverged())
  {
    transform = align.getFinalTransformation();
    ret = 0;
  }
  return ret;
}

/**
 * @brief Get best transform based on the Iterative Closest Point algorithm
 *
 */
int Map3DUtils::downsample(const PointCloudT::Ptr &src,
                           PointCloudT &dest,
                           float leaf)
{
  pcl::VoxelGrid<PointT> grid;
  grid.setLeafSize(leaf, leaf, leaf);
  grid.setInputCloud(src);
  grid.filter(dest); 
  
  return 0;
}

/**
 * @brief Get best transform based on the Iterative Closest Point algorithm
 *
 */
int Map3DUtils::estimateNormals(const PointCloudT::Ptr &src,
                                PointCloudNormalT &normals_src,
                                float radius)
{
  // Create an empty kdtree representation, and pass it to the normal estimation
  // object.
  // Its content will be filled inside the object, based on the given input
  // dataset (as no other search surface is given).
  pcl::search::KdTree<PointT>::Ptr tree =
          boost::make_shared<pcl::search::KdTree<PointT>>();
  pcl::NormalEstimation<PointT, NormalT> normal_est;
  normal_est.setSearchMethod(tree);
  normal_est.setInputCloud(src);
  normal_est.setRadiusSearch(radius);  // 50cm
  normal_est.compute(normals_src);
  
  return 0;
}

/**
 * @brief Get best transform based on the Iterative Closest Point algorithm
 *
 */
int Map3DUtils::estimateNormalsOMP(const PointCloudT::Ptr &src,
                                   PointCloudNormalT &normals_src, 
                                   float radius)
{
  pcl::NormalEstimationOMP<PointT, NormalT> ne;
  // Create an empty kdtree representation, and pass it to the normal estimation
  // object.
  // Its content will be filled inside the object, based on the given input
  // dataset (as no other search surface is given).
  pcl::search::KdTree<PointT>::Ptr tree =
          boost::make_shared<pcl::search::KdTree<PointT>>();

  ne.setNumberOfThreads(8);
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(radius);
  ne.setInputCloud(src);
  ne.compute(normals_src);
  Eigen::Vector4f centroid;
  compute3DCentroid(*src, centroid);
  ne.setViewPoint(centroid[0], centroid[1], centroid[2]);
  for (size_t i = 0; i < normals_src.size(); ++i) 
  {
      normals_src.points[i].normal_x *= -1;
      normals_src.points[i].normal_y *= -1;
      normals_src.points[i].normal_z *= -1;
  }
  
  return 0;
}

/**
 * @brief Get best transform based on the Iterative Closest Point algorithm
 *
 */
int Map3DUtils::findCorrespondences(const PointCloudFeatureT::Ptr &fpfhs_src,
                                    const PointCloudFeatureT::Ptr &fpfhs_tgt,
                                    pcl::Correspondences &all_correspondences)
{
  pcl::registration::CorrespondenceEstimation<FeatureT, FeatureT> est;
  est.setInputCloud(fpfhs_src);
  est.setInputTarget(fpfhs_tgt);
  est.determineReciprocalCorrespondences(all_correspondences);

  return 0;
}

/**
 * @brief Get best transform based on the Iterative Closest Point algorithm
 *
 */
int Map3DUtils::rejectBadCorrespondences(const pcl::CorrespondencesPtr &all_correspondences,
                                         const PointCloudT::Ptr &keypoints_src,
                                         const PointCloudT::Ptr &keypoints_tgt,
                                         pcl::Correspondences &remaining_correspondences,
                                         float max_dist)
{
  pcl::registration::CorrespondenceRejectorDistance rej;
  rej.setInputSource<PointT>(keypoints_src);
  rej.setInputTarget<PointT>(keypoints_tgt);
  rej.setMaximumDistance (max_dist);    // 1m
  rej.setInputCorrespondences(all_correspondences);
  rej.getCorrespondences(remaining_correspondences);

  return 0;
}
