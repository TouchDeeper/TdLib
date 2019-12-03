//
// Created by wang on 19-4-8.
//

#ifndef TDLIB_FILTER_HPP
#define TDLIB_FILTER_HPP

#include <pcl/common/common_headers.h>
#include "common_typedef.h"
namespace td{
    namespace pclib{
        /**
         *
         * @tparam PointType
         * @param cloud
         * @param field_name The field used to filter the cloud
         * @param min field min value
         * @param max field amx value
         */
        template <typename PointType>
        void PassFilter(typename pcl::PointCloud<PointType>::Ptr cloud, std::string field_name, float min, float max){
            pcl::PassThrough<PointType> pass;
            pass.setInputCloud(cloud);
            pass.setFilterFieldName(field_name);
            pass.setFilterLimits(min, max);
            pass.filter(*cloud);
        }
        /**
        * \brief uses point neighborhood statistics to filter outlier data
        * \details Points will be classified as inlier or outlier if their average neighbor distance is below or above this threshold respectively.
        * @param cloud input cloud
        * @param mean_k the number of nearest neighbors to use for mean distance estimation.
        * @param std_dev_mul_thresh stddev_mult The standard deviation multiplier.
        */
        template <typename PointType>
        void StatisticalFilter(typename pcl::PointCloud<PointType>::Ptr cloud, int mean_k, double std_dev_mul_thresh)
        {
            pcl::StatisticalOutlierRemoval<PointType> sor;
            sor.setInputCloud (cloud);
            sor.setMeanK (mean_k);
            sor.setStddevMulThresh (std_dev_mul_thresh);
            sor.filter (*cloud);
        }
    }//namespace pclib
} //namespace td
#endif //TDLIB_FILTER_H
