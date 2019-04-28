//
// Created by wang on 19-4-8.
//

#ifndef TDLIB_ENVELOPING_H
#define TDLIB_ENVELOPING_H

#include "common_typedef.h"

namespace td{
    namespace pclib{
        typedef pcl::PointXYZ PointType;
        /**
         * Compute the enveloping information of point cloud
         * @param cloud intput xyz cloud
         */
        void enveloping(pcl::PointCloud<PointType>::Ptr cloud);

        /**
         * Compute the enveloping information of point cloud
         * @param cloudWithNormal intput xyz with normal cloud
         */
        void enveloping(pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormal);
    }//namespace pclib
} //namespace td
#endif //TDLIB_ENVELOPING_H
