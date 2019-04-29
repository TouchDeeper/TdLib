//
// Created by wang on 19-4-29.
//
#include "keypoint.h"
#include "tools.h"
namespace td{
    namespace pclib{
        /**
         * \brief compute the ISS keypoints
         * @param model input cloud
         * @param model_keypoints ISS keypoints
         */
        void IssKeypoint(PointRGBACloudPtr model, PointRGBACloudPtr model_keypoints, bool border_estimation){
            //
        //  ISS3D parameters
        //
            double iss_salient_radius_;
            double iss_non_max_radius_;
            double iss_normal_radius_;
            double iss_border_radius_;
            double iss_gamma_21_ (0.975);
            double iss_gamma_32_ (0.975);
            int iss_min_neighbors_ (5);
            unsigned int iss_threads_ (4);

            pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());

        // Fill in the model cloud

            double model_resolution;
            model_resolution = computeCloudResolution(model);
        // Compute model_resolution

            iss_salient_radius_ = 6 * model_resolution;
            iss_non_max_radius_ = 4 * model_resolution;
            iss_normal_radius_ = 4 * model_resolution;
            iss_border_radius_ = 1 * model_resolution;

        //
        // Compute keypoints
        //
            pcl::ISSKeypoint3D<pcl::PointXYZRGBA, pcl::PointXYZRGBA> iss_detector;

            iss_detector.setSearchMethod (tree);
            iss_detector.setSalientRadius (iss_salient_radius_);
            iss_detector.setNonMaxRadius (iss_non_max_radius_);
            if(border_estimation)
            {
                iss_detector.setNormalRadius (iss_normal_radius_);
                iss_detector.setBorderRadius (iss_border_radius_);
            }
            iss_detector.setThreshold21 (iss_gamma_21_);
            iss_detector.setThreshold32 (iss_gamma_32_);
            iss_detector.setMinNeighbors (iss_min_neighbors_);
            iss_detector.setNumberOfThreads (iss_threads_);
            iss_detector.setInputCloud (model);
            iss_detector.compute (*model_keypoints);
        }
        void IssKeypoint(PointRGBCloudPtr model, PointRGBCloudPtr model_keypoints, bool border_estimation){
            //
        //  ISS3D parameters
        //
            double iss_salient_radius_;
            double iss_non_max_radius_;
            double iss_normal_radius_;
            double iss_border_radius_;
            double iss_gamma_21_ (0.975);
            double iss_gamma_32_ (0.975);
            int iss_min_neighbors_ (5);
            unsigned int iss_threads_ (4);

            pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());

        // Fill in the model cloud

            double model_resolution;
            model_resolution = computeCloudResolution(model);
        // Compute model_resolution

            iss_salient_radius_ = 6 * model_resolution;
            iss_non_max_radius_ = 4 * model_resolution;
            iss_normal_radius_ = 4 * model_resolution;
            iss_border_radius_ = 1 * model_resolution;

        //
        // Compute keypoints
        //
            pcl::ISSKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZRGB> iss_detector;

            iss_detector.setSearchMethod (tree);
            iss_detector.setSalientRadius (iss_salient_radius_);
            iss_detector.setNonMaxRadius (iss_non_max_radius_);

            if(border_estimation)
            {
                iss_detector.setNormalRadius (iss_normal_radius_);
                iss_detector.setBorderRadius (iss_border_radius_);
            }

            iss_detector.setThreshold21 (iss_gamma_21_);
            iss_detector.setThreshold32 (iss_gamma_32_);
            iss_detector.setMinNeighbors (iss_min_neighbors_);
            iss_detector.setNumberOfThreads (iss_threads_);
            iss_detector.setInputCloud (model);
            iss_detector.compute (*model_keypoints);
        }

    }//namespace pclib
}//namespace td
