//
// Created by wang on 19-4-8.
//

#ifndef TDLIB_ENVELOPING_H
#define TDLIB_ENVELOPING_H
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

#include <pcl/common/common_headers.h>

namespace td{

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
} //namespace td
#endif //TDLIB_ENVELOPING_H
