//
// Created by wang on 19-4-29.
//

#include "global_feature.h"


namespace td{
    namespace pclib{
        /**
         * Estimates a ESF feature for a point cloud cluster
         * @param cluster The cluster object containing the point cloud
         */
        void GlobalDescriptorEstimation (PointCloudPtr cluster, EsfDescriptor &esf_feature)
        {
            // Estimate global ESF feature for cluster
            EsfDescriptorEstimation feature_estimator;
            EsfDescriptorCloudPtr feature (new EsfDescriptorCloud ());
            feature_estimator.setInputCloud (cluster);
            feature_estimator.compute (*feature);

            esf_feature = feature->points[0];
        }
    }
}
