//
// Created by wang on 20-6-5.
//

#ifndef TDLIB_CLUSTER_SEGMENTATION_HPP
#define TDLIB_CLUSTER_SEGMENTATION_HPP

#include <pcl/point_cloud.h>
#include "common_typedef.h"
namespace td{
    namespace pclib{
        /**
           Performs euclidean cluster extraction on the point cloud. Only the largest cluster is kept. If the largets cluster is smaller than min_cluster_size (which means that no clusters were found) then the whole original point cloud is kept
           @param cloud The point cloud
        */
        template <typename PointType>
        void euclideanClusterExtraction (typename pcl::PointCloud<PointType>::Ptr cloud, double cluster_tolerance, int min_cluster_size, int max_cluster_size)
        {
            typename pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
            tree->setInputCloud (cloud);

            // Find cluster indices in the cloud
            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<PointType> ec;
            ec.setClusterTolerance (cluster_tolerance);
            ec.setMinClusterSize (min_cluster_size);
            ec.setMaxClusterSize (max_cluster_size);
            ec.setSearchMethod (tree);
            ec.setInputCloud (cloud);
            ec.extract (cluster_indices);

            // Extract the clusters
            std::vector<typename pcl::PointCloud<PointType>::Ptr> cloud_clusters;
            for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
            {
                typename pcl::PointCloud<PointType>::Ptr cloud_cluster (new pcl::PointCloud<PointType>);
                for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
                {
                    cloud_cluster->points.push_back (cloud->points[*pit]);
                }
                cloud_cluster->width = cloud_cluster->points.size ();
                cloud_cluster->height = 1;
                cloud_cluster->is_dense = true;
                cloud_clusters.push_back (cloud_cluster);
            }

            // Keep the largest cluster
            int max_size = 0;
            int index = 0;
            for (int i = 0; i < cloud_clusters.size(); i++)
            {
                if (cloud_clusters[i]->points.size() > max_size)
                {
                    index = i;
                    max_size = cloud_clusters[i]->points.size();
                }
            }

            if (max_size != 0)
            {
                copyPointCloud (*cloud_clusters[index], *cloud);
            }
        }
    }
}

#endif //TDLIB_CLUSTER_SEGMENTATION_HPP
