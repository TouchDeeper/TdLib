//
// Created by wang on 19-4-8.
//

#ifndef TDLIB_FILTER_H
#define TDLIB_FILTER_H

#include <pcl/common/common_headers.h>

namespace td{
    void colorFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud, int rmin = 0, int rmax = 255, int gmin = 0, int gmax = 255, int bmin = 0, int bmax = 255);
} //namespace td
#endif //TDLIB_FILTER_H
