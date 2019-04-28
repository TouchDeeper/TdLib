//
// Created by wang on 19-4-22.
//

#ifndef TDLIB_COMMON_TYPEDEF_H
#define TDLIB_COMMON_TYPEDEF_H

#include <iostream>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>

#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/features/normal_3d.h>

#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/segmentation/extract_clusters.h>

#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/centroid.h>

#include <pcl/surface/mls.h>

#include <Eigen/Core>



namespace td{
    namespace pclib{
        typedef pcl::ReferenceFrame RFType;
        typedef pcl::PointXYZ Point;
        typedef pcl::PointXYZRGB PointRGB;
        typedef pcl::PointNormal PointN;
        typedef pcl::PointCloud<pcl::Normal> NormalCloud;
        typedef pcl::PointCloud<pcl::Normal>::Ptr NormalCloudPtr;
        typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
        typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;
        typedef pcl::PointCloud<pcl::PointXYZRGB> PointRGBCloud;
        typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointRGBCloudPtr;
        typedef pcl::PointCloud<pcl::PointNormal> PointNCloud;
        typedef pcl::PointCloud<pcl::PointNormal>::Ptr PointNCloudPtr;
        typedef pcl::FPFHSignature33 FpfhDescriptorType;
        typedef pcl::PointCloud<FpfhDescriptorType> FpfhDescriptorCloud;
        typedef pcl::PointCloud<FpfhDescriptorType>::Ptr FpfhDescriptorCloudPtr;
        typedef std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > VectorMatrix4d;
        typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > VectorVector3d;
    }//namespace pclib
}//namespace td
#endif //TDLIB_COMMON_TYPEDEF_H
