//
// Created by wang on 19-4-13.
//

#ifndef TDLIB_TOOL_H
#define TDLIB_TOOL_H

#include "common_typedef.h"
namespace td{
    namespace pclib{
        void demean_cloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        /**
        * compute the cloud resolution
        * @param cloud input cloud
        * @return the cloud resolution
        */
        double computeCloudResolution (const PointCloudPtr &cloud);
        double computeCloudResolution (const PointRGBCloudPtr &cloud);
        double computeCloudResolution (const PointRGBACloudPtr &cloud);
    }//namespace pclib
}//namesoace td


#endif //TDLIB_TOOL_H
