//
// Created by wang on 19-4-8.
//

#ifndef TDLIB_FILTER_HPP
#define TDLIB_FILTER_HPP

#include <pcl/common/common_headers.h>
#include "common_typedef.h"
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>

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
        template <typename PointType>
        void PlaneSegment(typename pcl::PointCloud<PointType>::Ptr &scene_cloud, float distance_threshold, float height_min, float height_max) {
            //plane segmentation
            // Object for storing the plane model coefficients.
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            // Create the segmentation object.
            pcl::SACSegmentation<PointType> segmentation;
            segmentation.setInputCloud(scene_cloud);
            // Configure the object to look for a plane.
            segmentation.setModelType(pcl::SACMODEL_PLANE);
            // Use RANSAC method.
            segmentation.setMethodType(pcl::SAC_RANSAC);
            // Set the maximum allowed distance to the model.
            segmentation.setDistanceThreshold(distance_threshold);
            // Enable model coefficient refinement (optional).
            segmentation.setOptimizeCoefficients(true);

            pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);
            segmentation.segment(*planeIndices, *coefficients);

            if (planeIndices->indices.empty())
                std::cout << "Could not find any points that fitted the plane model." << std::endl;
            else
            {
                //                std::cerr << "Model coefficients: " << coefficients->values[0] << " "
//                          << coefficients->values[1] << " "
//                          << coefficients->values[2] << " "
//                          << coefficients->values[3] << std::endl;

                // Copy all inliers of the model to another cloud.
//                pcl::copyPointCloud<pcl::PointXYZRGB>(*cloud, inlierIndices, *inlierPoints);
                // Copy the points of the plane to a new cloud.
                pcl::ExtractIndices<PointType> extract;
                typename pcl::PointCloud<PointType>::Ptr plane(new pcl::PointCloud<PointType>);
                extract.setInputCloud(scene_cloud);
                extract.setIndices(planeIndices);
                extract.filter(*plane);

                // Retrieve the convex hull.
                typename pcl::PointCloud<PointType>::Ptr convexHull(new pcl::PointCloud<PointType>);
                pcl::ConvexHull<PointType> hull;
                hull.setInputCloud(plane);
                // Make sure that the resulting hull is bidimensional.
                hull.setDimension(2);
                hull.reconstruct(*convexHull);

                // Redundant check.
                if (hull.getDimension() == 2)
                {
                    // Prism object.
                    pcl::ExtractPolygonalPrismData<PointType> prism;
                    prism.setInputCloud(scene_cloud);
                    prism.setInputPlanarHull(convexHull);
                    // First parameter: minimum Z value. Set to 0, segments objects lying on the plane (can be negative).
                    // Second parameter: maximum Z value, set to 10cm. Tune it according to the height of the objects you expect.
                    prism.setHeightLimits(height_min, height_max);


                    pcl::PointIndices::Ptr objectIndices(new pcl::PointIndices);

                    prism.segment(*objectIndices);

                    // Get and show all points retrieved by the hull.
                    extract.setIndices(objectIndices);
                    extract.filter(*scene_cloud);
                }
            }
        }
    }//namespace pclib
} //namespace td
#endif //TDLIB_FILTER_H
