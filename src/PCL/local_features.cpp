//
// Created by wang on 19-5-9.
//
#include "local_features.h"
namespace td{
    namespace pclib{
        /**
         * compute the local descriptor
         * @param cluster input cloud
         * @param fpfh_feature fpfh feature
         * @param feature_search_radius the radius to compute the descriptors
         */
        void LocalDescriptorEstimation (PointNCloudPtr cluster, FpfhDescriptorCloudPtr fpfh_feature, double feature_search_radius)
        {
            pcl::FPFHEstimationOMP<PointN, PointN, FpfhDescriptor> feature_estimator;
            feature_estimator.setRadiusSearch(feature_search_radius);
            feature_estimator.setInputCloud(cluster);
            feature_estimator.setInputNormals(cluster);
//            feature_estimator.setSearchSurface(cluster);
            feature_estimator.compute(*fpfh_feature);
        }

    }
}

