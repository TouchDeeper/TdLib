//
// Created by wang on 19-4-13.
//

#ifndef TDLIB_TOOL_H
#define TDLIB_TOOL_H

#include "common_typedef.h"
#include <pcl/features/normal_3d_omp.h>
namespace td{
    namespace pclib{
        void estimate_normals (	PointCloudPtr cloud_xyz,
                                   PointNCloudPtr cloud_N,
                                   float vp_x,
                                   float vp_y,
                                   float vp_z,
                                   int k_neighbor);
        void estimate_normals (	PointCloudPtr cloud_xyz,
                                   PointNCloudPtr cloud_N,
                                   float vp_x,
                                   float vp_y,
                                   float vp_z,
                                   float search_radius);
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
         * @param source the source view
         * @param target the target view
         * @param t_T_s the transformation t_T_s
         */
        void ShowAlignResult(const PointNCloudPtr source, const PointNCloudPtr target, const Eigen::Matrix4d t_T_s,std::string viewer_name,ColorHandlerPointN target_color, ColorHandlerPointN source_color,std::vector<int> window_size = std::vector<int>(2,0),double point_size=0.1,std::vector<int> background_color = std::vector<int>(3,0));
        void ShowBeforeAndAfterAlign(const PointNCloudPtr source, const PointNCloudPtr target, const Eigen::Matrix4d t_T_s_before, const Eigen::Matrix4d t_T_s_after,std::string viewer_name,ColorHandlerPointN target_color, ColorHandlerPointN source_color,std::vector<int> window_size = std::vector<int>(2,0),double point_size=0.1,std::vector<int> background_color = std::vector<int>(3,0));

        /**
         *
         * @param cloud
         * @param scale coordinate system scale
         */
        void quickShowCloud(const PointCloudPtr& cloud, double scale);
        /**
        * get infomation after the souce be registered to the target
        * @param input_source source cloud
        * @param target_cloud_kdtree_
        * @param inlier_squared_threshold squared distance threshold for inlier check
        * @param inlier_fraction inlier_size/input_source->size
        * @param inlier_size
        * @param error
        * @param tg_T_sr transformation from source to target
        */
        void getFitness(PointNCloudPtr input_source, pcl::search::KdTree<pcl::PointNormal> target_cloud_kdtree_,
                        double inlier_squared_threshold,
                        double& inlier_fraction, int& inlier_size,
                        float& error, Eigen::Matrix4d tg_T_sr);
        /**
        * get outlier cloud after the souce be registered to the target
        * @param input_source source cloud
        * @param target_cloud_kdtree_
        * @param inlier_squared_threshold squared distance threshold for inlier check
        * @param tg_T_sr transformation from source to target
        * @return outlier_cloud ptr
        */
        PointNCloudPtr getOutlierCloud(PointNCloudPtr input_source, pcl::search::KdTree<pcl::PointNormal> target_cloud_kdtree_,
                                       double inlier_squared_threshold,Eigen::Matrix4d tg_T_sr);

    }//namespace pclib
}//namesoace td


#endif //TDLIB_TOOL_H
