//
// Created by wang on 19-4-13.
//

#ifndef TDLIB_TOOL_H
#define TDLIB_TOOL_H

#include "common_typedef.h"
namespace td{
    namespace pclib{
        void demean_cloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        /**
        * compute the cloud resolution
        * @param cloud input cloud
        * @return the cloud resolution
        */
        double computeCloudResolution (const PointCloudPtr &cloud);
        double computeCloudResolution (const PointRGBCloudPtr &cloud);
        double computeCloudResolution (const PointRGBACloudPtr &cloud);
        /**
         * show the align result
         * @param object the object view
         * @param scene the scene view
         * @param transformation the transformation s_T_o
         */
        void ShowAlignResult(const PointNCloudPtr model, const PointNCloudPtr scene, const Eigen::Matrix4d transformation,std::string viewer_name,ColorHandlerPointN scene_view_color, ColorHandlerPointN model_view_color,std::vector<int> window_size = std::vector<int>(2,0),double point_size=0.1,std::vector<int> background_color = std::vector<int>(3,0));
    }//namespace pclib
}//namesoace td


#endif //TDLIB_TOOL_H
