//
// Created by wang on 19-4-29.
//

#ifndef TDLIB_GLOBAL_FEATURE_H
#define TDLIB_GLOBAL_FEATURE_H

#include "common_typedef.h"
namespace td{
    namespace pclib{
        /**
         * Estimates a ESF feature for a point cloud cluster
         * @param cluster The cluster object containing the point cloud
         */
        void GlobalDescriptorEstimation (PointCloudPtr cluster, EsfDescriptor &esf_feature);
    }
}
#endif //TDLIB_GLOABAL_FEATURE_H
