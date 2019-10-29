//
// Created by wang on 19-4-8.
//

#ifndef TDLIB_FILTER_H
#define TDLIB_FILTER_H

#include <pcl/common/common_headers.h>
#include "common_typedef.h"
namespace td{
    namespace pclib{
        void colorFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud, int rmin = 0, int rmax = 255, int gmin = 0, int gmax = 255, int bmin = 0, int bmax = 255);

        /**
         * downsampling the point cloud
         * @param model the input cloud
         * @param leaf_size set the size of every square voxel
         */
        void DownsamplingSquareLeaf(PointCloudPtr model, float leaf_size);
        void DownsamplingSquareLeaf(PointNCloudPtr model, float leaf_size);
        /**
         * upsamling the cloud
         * @param model input cloud
         * @param search_radius Use all neighbors in this radius.
         * @param sampling_radius Radius around each point, where the local plane will be sampled.
         * @param step_size Sampling step size. Bigger values will yield less (if any) new points.
         */
        void Upsampling(PointCloudPtr model, float search_radius, float sampling_radius, float step_size);
         /**
         * \brief uses point neighborhood statistics to filter outlier data
         * \details Points will be classified as inlier or outlier if their average neighbor distance is below or above this threshold respectively.
         * @param cloud input cloud
         * @param mean_k the number of nearest neighbors to use for mean distance estimation.
         * @param std_dev_mul_thresh stddev_mult The standard deviation multiplier.
         */
        void StatisticalFilter(PointCloudPtr cloud, int mean_k, double std_dev_mul_thresh);
        void StatisticalFilter(PointRGBCloudPtr cloud, int mean_k, double std_dev_mul_thresh);
    }//namespace pclib
} //namespace td
#endif //TDLIB_FILTER_H
