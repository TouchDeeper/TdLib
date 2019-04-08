//
// Created by wang on 19-4-8.
//
#include "filter.h"
#include <pcl/filters/conditional_removal.h>
namespace td{

    void colorFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud, int rmin, int rmax, int gmin, int gmax, int bmin, int bmax)
    {

        pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
        //the point's r value must be less than (LT) 256.
        color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::LT, rmax)));
        //the point's r value must be greater than (GT) 200.
        color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::GT, rmin)));
        color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::LT, gmax)));
        color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::GT, gmin)));
        color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::LT, bmax)));
        color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::GT, bmin)));

        // build the filter
        pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
        condrem.setCondition (color_cond);
        condrem.setInputCloud (inputCloud);
        condrem.setKeepOrganized(true);

        // apply filter
        condrem.filter (*inputCloud);
    }

}