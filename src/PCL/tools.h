//
// Created by wang on 19-4-13.
//

#ifndef TDLIB_TOOL_H
#define TDLIB_TOOL_H
#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>
namespace td{
    void demean_cloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
}//namesoace td


#endif //TDLIB_TOOL_H
