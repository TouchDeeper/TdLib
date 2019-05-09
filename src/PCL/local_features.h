//
// Created by wang on 19-5-9.
//

#ifndef TDLIB_LOCAL_FEATURES_H
#define TDLIB_LOCAL_FEATURES_H

#include "common_typedef.h"
namespace td{
    namespace pclib{
        /**
         * compute the local descriptor
         * @param cluster input cloud
         * @param fpfh_feature fpfh feature
         * @param feature_search_radius the radius to compute the descriptors
         */
        void LocalDescriptorEstimation (PointNCloudPtr cluster, FpfhDescriptorCloudPtr fpfh_feature, double feature_search_radius);
    }
}
#endif //TDLIB_LOCAL_FEATURES_H
