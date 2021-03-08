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

#ifndef _MAP_3D_UTILS_H
#define _MAP_3D_UTILS_H


// Nguyen2012 - Modeling kinect sensor noise for improved 3D reconstruction and tracking
#define SENSORNOISEMODEL(x)  (0.0012 + 0.0019 * (x - 0.4) * (x - 0.4))

#include <Eigen/Dense>

#include <boost/thread.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/fast_bilateral.h>

#include <pcl/features/integral_image_normal.h>

#include <pcl/filters/normal_space.h>

#include <pcl/registration/correspondence_estimation.h>

#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>

#include <pcl/registration/transformation_estimation_point_to_plane_weighted.h>


namespace ramon_slam3d
{
	/**
	* @class Map3DUtils
	* @brief Clase para representar celdas.
	*/

  template <typename PointSourceT, typename PointT>
  class Map3DUtils
  {
  public:
    /**
     * @brief IMU topic callback function
     *
     * @param scanptr Pointer to imu data
     */
    Map3DUtils(){};

    /**
     * @brief IMU topic callback function
     *
     * @param scanptr Pointer to imu data
     */
    ~Map3DUtils(){};
  
    /**
     * @brief IMU topic callback function
     *
     * @param scanptr Pointer to imu data
     */
    int 
    estimateFinalTransform(const boost::shared_ptr<const pcl::PointCloud<PointT> > &src,
                           const boost::shared_ptr<const pcl::PointCloud<PointT> > &tgt,
                           pcl::CorrespondencesPtr &all_correspondences,
                           std::vector<double> &weights,
                           Eigen::Matrix4d &transform)
    {
      pcl::registration::TransformationEstimationPointToPlaneWeighted<PointT, PointT, double> te;
      te.setUseCorrespondenceWeights(true);
      te.setWeights(weights);
      te.estimateRigidTransformation(*src, *tgt, *all_correspondences, transform);
      
      return 0;
    }



    // https://pointclouds.org/documentation/tutorials/normal_estimation_using_integral_images.html
    /**
     * @brief IMU topic callback function
     *
     * @param scanptr Pointer to imu data
     */
    int 
    estimateIntegralImageNormals(const boost::shared_ptr<const pcl::PointCloud<PointSourceT> > &src,
                                pcl::PointCloud<PointT> &normals_src,
                                float charge_factor = 0.02f,
                                float smoothing_size = 10.0f)
    {
      pcl::copyPointCloud(*src, normals_src);

      pcl::IntegralImageNormalEstimation<PointSourceT, PointT> ne;
      // ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
      ne.setNormalEstimationMethod (ne.AVERAGE_DEPTH_CHANGE);
      ne.setMaxDepthChangeFactor(charge_factor);
      ne.setNormalSmoothingSize(smoothing_size);
      ne.setInputCloud(src);
      ne.compute(normals_src);

      return 0;
    }    

    /**
     * @brief IMU topic callback function
     *
     * @param scanptr Pointer to imu data
     */
    int 
    computeNormalSpaceSampling(const boost::shared_ptr<const pcl::PointCloud<PointT> > &src,
                               std::vector<int> &walls_indices,
                               float sample,
                               float bins = 2,
                               float seed = 0)
    {
      pcl::NormalSpaceSampling<PointT, PointT> normal_space_sampling;
      normal_space_sampling.setInputCloud (src);
      normal_space_sampling.setNormals (src);
      normal_space_sampling.setBins (bins, bins, bins);
      normal_space_sampling.setSeed (seed);
      normal_space_sampling.setSample (sample);
      normal_space_sampling.filter(walls_indices);

      return 0;
    }

    /**
     * @brief IMU topic callback function
     *
     * @param scanptr Pointer to imu data
     */                                     
    int 
    filterAndUpsample(const boost::shared_ptr<const pcl::PointCloud<PointSourceT> > &src,
                      pcl::PointCloud<PointSourceT> &dest,
                      float sigma_s = 5,
                      float sigma_r = 0.005f)
    {
      pcl::FastBilateralFilter<PointSourceT> bilateral_filter;
      bilateral_filter.setInputCloud(src);
      bilateral_filter.setSigmaS(sigma_s);
      bilateral_filter.setSigmaR(sigma_r);
      bilateral_filter.filter(dest); 
      
      return 0;
    }

    /**
     * @brief IMU topic callback function
     *
     * @param scanptr Pointer to imu data
     */
    int 
    filterAndUpsample(const boost::shared_ptr<const pcl::PointCloud<PointT> > &src,
                      pcl::PointCloud<PointT> &dest,
                      float sigma_s = 5,
                      float sigma_r = 0.005f)
    {
      pcl::FastBilateralFilter<PointT> bilateral_filter;
      bilateral_filter.setInputCloud(src);
      bilateral_filter.setSigmaS(sigma_s);
      bilateral_filter.setSigmaR(sigma_r);
      bilateral_filter.filter(dest); 
      
      return 0;
    }

    /**
     * @brief IMU topic callback function
     *
     * @param scanptr Pointer to imu data
     */                           
    int 
    findCorrespondences(const boost::shared_ptr<const pcl::PointCloud<PointT> > &src,
                        const boost::shared_ptr<const pcl::PointCloud<PointT> > &tgt,
                        pcl::Correspondences &all_correspondences,
                        float max_dist = 1.0)
    {
      pcl::registration::CorrespondenceEstimation<PointT, PointT> est;

      est.setInputSource(src);
      est.setInputTarget(tgt);
      est.determineReciprocalCorrespondences(all_correspondences);
      // est.determineCorrespondences(all_correspondences, max_dist);

      return 0;
    }

    /**
     * @brief IMU topic callback function
     *
     * @param scanptr Pointer to imu data
     */
    int 
    rejectBadCorrespondencesNormals(const pcl::CorrespondencesPtr &all_correspondences,
                                    const boost::shared_ptr<const pcl::PointCloud<PointT> > &src,
                                    const boost::shared_ptr<const pcl::PointCloud<PointT> > &src_normal,
                                    const boost::shared_ptr<const pcl::PointCloud<PointT> > &tgt,
                                    const boost::shared_ptr<const pcl::PointCloud<PointT> > &tgt_normal,
                                    std::vector<double> &weights,
                                    pcl::Correspondences &remaining_correspondences,
                                    float max_angle = M_PI/4,
                                    float median_factor = 8.79241104)
    {
      pcl::registration::CorrespondenceRejectorMedianDistance rej;
      rej.setMedianFactor (median_factor);
      rej.setInputCorrespondences (all_correspondences);

      rej.getCorrespondences (remaining_correspondences);
      
      pcl::CorrespondencesPtr remaining_correspondences_temp (new pcl::Correspondences);
      rej.getCorrespondences (*remaining_correspondences_temp);

      pcl::registration::CorrespondenceRejectorSurfaceNormal rej_normals;
      rej_normals.setThreshold(std::acos(max_angle));
      rej_normals.initializeDataContainer<PointT, PointT>();
      rej_normals.setInputSource<PointT> (src);
      rej_normals.setInputNormals<PointT, PointT> (src_normal);
      rej_normals.setInputTarget<PointT> (tgt);
      rej_normals.setTargetNormals<PointT, PointT> (tgt_normal);
      rej_normals.setInputCorrespondences(remaining_correspondences_temp);
      rej_normals.getCorrespondences(remaining_correspondences);

      // Weights estimation
      // Nguyen2012 - Modeling kinect sensor noise for improved 3D reconstruction and tracking
      // Holz2015 - Registration with the point cloud library: A modular framework for aligning in 3-D
      weights.resize(remaining_correspondences.size());
      for(int i = 0; i < remaining_correspondences.size(); i++)
      {
        PointT src_idx = src->points[remaining_correspondences[i].index_query];
        PointT tgt_idx = tgt->points[remaining_correspondences[i].index_match];
        weights[i] = std::max(SENSORNOISEMODEL(src_idx.z), SENSORNOISEMODEL(tgt_idx.z));
      }
      return 0;
    }

    /**
     * @brief IMU topic callback function
     *
     * @param scanptr Pointer to imu data
     */
    int
    registrationPipelineHolz(const boost::shared_ptr<const pcl::PointCloud<PointSourceT> > &cloud_reg_filtered,
                            const boost::shared_ptr<const pcl::PointCloud<PointSourceT> > &cloud_prev_filtered,
                            Eigen::Matrix4d &transform,
                            int iterations = 10)
    {
      boost::shared_ptr<pcl::PointCloud<PointT> > cloud_reg_normals(new pcl::PointCloud<PointT>);
      boost::shared_ptr<pcl::PointCloud<PointT> > cloud_prev_normals(new pcl::PointCloud<PointT>);

      boost::shared_ptr<pcl::PointCloud<PointT> > cloud_reg_normals_aux(new pcl::PointCloud<PointT>);
      boost::shared_ptr<pcl::PointCloud<PointT> > cloud_prev_normals_aux(new pcl::PointCloud<PointT>);

      std::vector<int> cloud_reg_indices_ign;
      std::vector<int> cloud_prev_indices_ign;


      pcl::CorrespondencesPtr all_correspondences (new pcl::Correspondences), 
                              good_correspondences (new pcl::Correspondences);
                              
      std::vector<double> weights;

      std::vector<boost::thread*> sample_pc_threads;

      Eigen::Matrix4d transform_prev, transform_reg;


      transform_prev << 1, 0, 0, 0,
                        0, 1, 0, 0,
                        0, 0, 1, 0,
                        0, 0, 0, 1;

      
      // Get the sampled pointclouds (optimized with threads)
      sample_pc_threads.push_back(
          new boost::thread(&Map3DUtils::getSampledCloud, this, cloud_reg_filtered, cloud_reg_normals));
      sample_pc_threads.push_back(
          new boost::thread(&Map3DUtils::getSampledCloud, this, cloud_prev_filtered, cloud_prev_normals));

      // delete created threads
      for (int i = 0; i < sample_pc_threads.size(); i++)
      {
        sample_pc_threads[i]->join();
        delete sample_pc_threads[i];
      }
      
      for(int i = 0; i < iterations; i++)
      {
        // Second time
        pcl::transformPointCloud(*cloud_reg_normals, *cloud_reg_normals_aux, transform_prev);

        // Find correspondences
        this->findCorrespondences(cloud_reg_normals_aux, cloud_prev_normals, *all_correspondences);

        
        // Reject bad correspondences
        this->rejectBadCorrespondencesNormals(all_correspondences, cloud_reg_normals_aux, cloud_reg_normals_aux, cloud_prev_normals, cloud_prev_normals, weights, *good_correspondences);

        if(good_correspondences->size() < 4)
          return -1;

        // Find transform
        this->estimateFinalTransform(cloud_reg_normals_aux, cloud_prev_normals, good_correspondences, weights, transform_reg);

        transform_reg *= transform_prev;
        transform_prev = transform_reg;
      }
      
      transform = transform_reg;

      return 0;
    }
  
  protected:
    int
    getSampledCloud(const boost::shared_ptr<const pcl::PointCloud<PointSourceT> > &cloud_in, boost::shared_ptr<pcl::PointCloud<PointT> > &cloud_out)
    {
      boost::shared_ptr<pcl::PointCloud<PointT> > cloud_aux(new pcl::PointCloud<PointT>);
      
      std::vector<int> cloud_indices;

      std::vector<int> cloud_indices_ign;

      // Estimate normals
      this->estimateIntegralImageNormals(cloud_in, *cloud_aux);

      // Sample
      this->computeNormalSpaceSampling(cloud_aux, cloud_indices, cloud_aux->width);

      // Get PCs with indices only
      pcl::copyPointCloud (*cloud_aux, cloud_indices, *cloud_aux);

      // Remove NANs
      pcl::removeNaNFromPointCloud(*cloud_aux, *cloud_out, cloud_indices_ign);

      return 0;
    }
  private:

  };
};
#endif //_MAP_3D_UTILS_H