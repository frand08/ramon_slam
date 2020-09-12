/**
 * @file map3d_utils.h
 * @author Francisco A. Dominguez (dominguezfranciscoa@gmail.com)
 * @brief
 * @version 0.1
 * @date 2020-09-09
 *
 * @copyright Copyright (c) 2020
 *
 */

#ifndef _map3d_utils_h
#define _map3d_utils_h

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>

#include <pcl/search/kdtree.h>
#include <pcl/filters/uniform_sampling.h>

#include <pcl/keypoints/sift_keypoint.h>

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_distance.h>

#include <Eigen/Dense>


namespace ramon_slam3d
{
	/**
	* @class Map3DUtils
	* @brief Clase para representar celdas.
	*/
  typedef pcl::PointXYZRGB PointT;
  typedef pcl::PointCloud<PointT> PointCloudT;
  typedef pcl::PointNormal NormalT;
  typedef pcl::PointCloud<NormalT> PointCloudNormalT;
  typedef pcl::FPFHSignature33 FeatureT;
  typedef pcl::PointCloud<FeatureT> PointCloudFeatureT;
  typedef pcl::PointWithScale PointScaleT;
  typedef pcl::PointCloud<PointScaleT> PointCloudScaleT;

  class Map3DUtils
  {
  public:
    Map3DUtils();
    ~Map3DUtils();
    int downsample(const PointCloudT::Ptr &src,
                   PointCloudT &dest,
                   float leaf = 0.05);
                  
    int estimateFPFH(const PointCloudT::Ptr &src,
             const PointCloudNormalT::Ptr &normals_src,
             const PointCloudT::Ptr &keypoints_src,
             PointCloudFeatureT &fpfhs_src,
             float radius = 1.0);

    int estimateKeypoints(const PointCloudT::Ptr &src,
                          PointCloudT &keypoints_src,
                          float radius = 1.0);

    int estimateNormals(const PointCloudT::Ptr &src,
                        PointCloudNormalT &normals_src,
                        float radius = 0.5);

    int estimateNormalsOMP(const PointCloudT::Ptr &src,
                          PointCloudNormalT &normals_src, 
                          float radius = 0.5);

    int estimateSIFTKeypoints(const PointCloudT::Ptr &src,
                              PointCloudT &keypoints_src,
                              const float min_scale = 0.005, 
                              const int n_octaves = 10,
                              const int n_scales_per_octave = 8,
                              const float min_contrast = 1.5);

    int findCorrespondences(const PointCloudFeatureT::Ptr &fpfhs_src,
                            const PointCloudFeatureT::Ptr &fpfhs_tgt,
                            pcl::Correspondences &all_correspondences);

    int icp(const PointCloudT::Ptr &cloud_prev, 
            const PointCloudT::Ptr &cloud_reg, 
            Eigen::Matrix4f &transform,
            float max_corresp_dist = 0.05,
            uint32_t max_iterations = 100,
            float transformation_epsilon = 1e-8,
            float euclidean_fitness = 1);

    int ransac(const PointCloudT::Ptr &cloud_prev,
               const PointCloudFeatureT::Ptr &cloud_prev_features,
               const PointCloudT::Ptr &cloud_reg,
               const PointCloudFeatureT::Ptr &cloud_reg_features,
               Eigen::Matrix4f &transform,
               float leaf = 0.1);

    int rejectBadCorrespondences(const pcl::CorrespondencesPtr &all_correspondences,
                                 const PointCloudT::Ptr &keypoints_src,
                                 const PointCloudT::Ptr &keypoints_tgt,
                                 pcl::Correspondences &remaining_correspondences,
                                 float max_dist = 1.0);
  protected:

  private:

  };
};
#endif //_map3d_utils_h