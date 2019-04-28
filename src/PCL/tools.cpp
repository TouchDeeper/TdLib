//
// Created by wang on 19-4-13.
//
#include "tools.h"

namespace td{
    namespace pclib{
        /**
        Demeans the point cloud
        @param cloud The point cloud
        */
        void demean_cloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
        {
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid (*cloud, centroid);
            pcl::demeanPointCloud<pcl::PointXYZ> (*cloud, centroid, *cloud);
        }
    }

}
