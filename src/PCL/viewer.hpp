//
// Created by wang on 20-6-5.
//

#ifndef TDLIB_VIEWER_HPP
#define TDLIB_VIEWER_HPP

#include "common_typedef.h"
namespace td{
    namespace pclib{
        template <typename PointType>
        void quickShowCloud(const typename pcl::PointCloud<PointType>::Ptr& cloud){
            pcl::visualization::CloudViewer viewer("Cloud Viewer");
            viewer.showCloud(cloud);
            while(!viewer.wasStopped());
        }
        template <typename PointType>
        void quickShowCloud(const typename pcl::PointCloud<PointType>::Ptr cloud, double scale){
            pcl::visualization::PCLVisualizer viewer ("cloud viewer");
            viewer.addPointCloud<PointType>(cloud);
            viewer.addCoordinateSystem(scale);
            viewer.spin();
        }
    }
}

#endif //TDLIB_VIEWER_HPP
