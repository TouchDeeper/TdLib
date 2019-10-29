//
// Created by wang on 19-4-8.
//

#include "filter.h"

namespace td{
    namespace pclib{

        void colorFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud, int rmin, int rmax, int gmin, int gmax, int bmin, int bmax)
        {

            pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
            //the point's r value must be less than (LT) 256.
            if(rmax < 255)
                color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::LT, rmax)));
            //the point's r value must be greater than (GT) 200.
            if(rmin > 0)
                color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::GT, rmin)));
            if(gmax < 255)
                color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::LT, gmax)));
            if(gmin > 0)
                color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::GT, gmin)));
            if(bmax < 255)
                color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::LT, bmax)));
            if(bmin > 0)
                color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::GT, bmin)));

            // build the filter
            pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
            condrem.setCondition (color_cond);
            condrem.setInputCloud (inputCloud);
            condrem.setKeepOrganized(true);

            // apply filter
            condrem.filter (*inputCloud);
        }

        /**
         * downsampling the point cloud
         * @param model the input cloud
         * @param leaf_size set the size of every square voxel
         */
        void DownsamplingSquareLeaf(PointCloudPtr model, float leaf_size)
        {
            std::cerr << "before downsamplling, cloud has" << (*model).points.size () << " data points." << std::endl;
            pcl::VoxelGrid<pcl::PointXYZ> filter;
            filter.setInputCloud(model);
            // We set the size of every voxel to be 1x1x1cm
            // (only one point per every cubic centimeter will survive).
            filter.setLeafSize(leaf_size, leaf_size, leaf_size);
            filter.filter(*model);
            std::cerr << "after downsamplling, cloud has" << (*model).points.size () << " data points." << std::endl;
        }
        void DownsamplingSquareLeaf(PointNCloudPtr model, float leaf_size)
        {
            std::cerr << "before downsamplling, cloud has" << (*model).points.size () << " data points." << std::endl;
            pcl::VoxelGrid<pcl::PointNormal> filter;
            filter.setInputCloud(model);
            // We set the size of every voxel to be 1x1x1cm
            // (only one point per every cubic centimeter will survive).
            filter.setLeafSize(leaf_size, leaf_size, leaf_size);
            filter.filter(*model);
            std::cerr << "after downsamplling, cloud has" << (*model).points.size () << " data points." << std::endl;
        }

        /**
         * upsamling the cloud
         * @param model input cloud
         * @param search_radius Use all neighbors in this radius.
         * @param sampling_radius Radius around each point, where the local plane will be sampled.
         * @param step_size Sampling step size. Bigger values will yield less (if any) new points.
         */
        void Upsampling(PointCloudPtr model, float search_radius, float sampling_radius, float step_size)
        {
            std::cerr << "before upsamplling, cloud has" << (*model).points.size () << " data points." << std::endl;
            // Filtering object.
            pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> filter;
            filter.setInputCloud(model);
            // Object for searching.
            pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;
            filter.setSearchMethod(kdtree);

            filter.setSearchRadius(search_radius);
            // Upsampling method. Other possibilites are DISTINCT_CLOUD, RANDOM_UNIFORM_DENSITY
            // and VOXEL_GRID_DILATION. NONE disables upsampling. Check the API for details.
            filter.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);

            filter.setUpsamplingRadius(sampling_radius);

            filter.setUpsamplingStepSize(step_size);

            filter.process(*model);
            std::cerr << "after upsamplling, cloud has" << (*model).points.size () << " data points." << std::endl;
        }
        /**
         * \brief uses point neighborhood statistics to filter outlier data
         * \details Points will be classified as inlier or outlier if their average neighbor distance is below or above this threshold respectively.
         * @param cloud input cloud
         * @param mean_k the number of nearest neighbors to use for mean distance estimation.
         * @param std_dev_mul_thresh stddev_mult The standard deviation multiplier.
         */
        void StatisticalFilter(PointCloudPtr cloud, int mean_k, double std_dev_mul_thresh)
        {
            pcl::StatisticalOutlierRemoval<Point> sor;
            sor.setInputCloud (cloud);
            sor.setMeanK (mean_k);
            sor.setStddevMulThresh (std_dev_mul_thresh);
            sor.filter (*cloud);
        }
        void StatisticalFilter(PointRGBCloudPtr cloud, int mean_k, double std_dev_mul_thresh)
        {
            pcl::StatisticalOutlierRemoval<PointRGB> sor;
            sor.setInputCloud (cloud);
            sor.setMeanK (mean_k);
            sor.setStddevMulThresh (std_dev_mul_thresh);
            sor.filter (*cloud);
        }
    }//namespace pclib
}//namespace td