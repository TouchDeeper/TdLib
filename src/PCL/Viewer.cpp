//
// Created by wang on 19-9-11.
//

#include "Viewer.h"
namespace td{
    namespace pclib{
        void Viewer::ShowCorrespondence(const std::pair<std::vector<std::vector<int>>,float>* correspondence, const PointNCloudPtr source_view, const PointNCloudPtr target_view,
        const Eigen::Matrix4d transformations,ColorHandlerPointN target_view_color, ColorHandlerPointN source_view_color,std::vector<int> window_size,double point_size,std::vector<int> background_rgb)
        {

            pcl::visualization::PCLVisualizer viewer ("Correspondence Grouping");

            if(!(window_size[0] == 0 || window_size[1] == 0))
                viewer.setSize(window_size[0],window_size[1]);
            int vp_1;//two viewport, left for showing align result, right for showing the keypoint correspondence
            int vp_2;
            viewer.createViewPort (0.0, 0, 0.5, 1.0, vp_1);
            viewer.createViewPort (0.5, 0, 1.0, 1.0, vp_2);
            viewer.setBackgroundColor(background_rgb[0],background_rgb[1],background_rgb[2],vp_1);
            viewer.setBackgroundColor(background_rgb[0],background_rgb[1],background_rgb[2],vp_2);
            viewer.addText ("align", 10, 10, 18, 1.0, 1.0, 1.0, "text1", vp_1);

            viewer.addCoordinateSystem (0.03);
            target_view_color.setInputCloud(target_view);
//            viewer.addPointCloud<PointN>(scene_view,ColorHandlerPointN (scene_view,target_view_color[0], target_view_color[1], target_view_color[2]),"scene_view_vp1",vp_1);
            viewer.addPointCloud<PointN>(target_view,target_view_color,"scene_view_vp1",vp_1);
            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "scene_view_vp1",vp_1);

            PointCloudPtr sampling_keypoints(new PointCloud);
            PointCloudPtr match_keypoints(new PointCloud);
            //viewport 1
            viewer.removePointCloud("model_view");
            PointNCloudPtr model_view_transformed(new PointNCloud);
//            std::cout<<transformations<<std::endl;
            pcl::transformPointCloud(*source_view, *model_view_transformed, transformations);
            source_view_color.setInputCloud(model_view_transformed);
//            viewer.addPointCloud<PointN>(model_view_transformed,ColorHandlerPointN (model_view_transformed,source_view_color[0], source_view_color[1], source_view_color[2]),"model_view",vp_1);
            viewer.addPointCloud<PointN>(model_view_transformed,source_view_color,"model_view",vp_1);
            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "model_view",vp_1);

            //viewport 2
            //clear viewport2
            viewer.removeAllPointClouds(vp_2);
            viewer.removeAllShapes(vp_2);

            viewer.addText ("correspondence", 10, 10, 18, 1.0, 1.0, 1.0, "text2", vp_2);
//            viewer.addPointCloud<PointN>(scene_view,ColorHandlerPointN (target_view_color[0], target_view_color[1], target_view_color[2]),"scene_view_vp2",vp_2);
            viewer.addPointCloud<PointN>(target_view,target_view_color,"scene_view_vp2",vp_2);
            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "scene_view_vp2",vp_2);

            PointNCloudPtr model_view_transformed_translation(new PointNCloud);
            Eigen::Matrix4d translation = Eigen::Matrix4d::Identity();
            translation(0,3) = 0.05;
            translation(1,3) = 0.05;
            translation(2,3) = 0.05;
            pcl::transformPointCloud(*model_view_transformed, *model_view_transformed_translation,translation);//translate the model_view for drawing the correspondence line

            source_view_color.setInputCloud(model_view_transformed_translation);
//            viewer.addPointCloud<PointN>(model_view_transformed_translation,ColorHandlerPointN (model_view_transformed,source_view_color[0], source_view_color[1], source_view_color[2]),"model_view_translation",vp_2);
            viewer.addPointCloud<PointN>(model_view_transformed_translation,source_view_color,"model_view_translation",vp_2);
            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "model_view_translation",vp_2);

            for (int i = 0; i < (*correspondence).first[0].size(); ++i) {
                std::stringstream ss_line;
                ss_line << "correspondence_line" << i;
                pcl::PointXYZ model_point;
                model_point.x = (model_view_transformed_translation->at(correspondence->first[0][i])).x;
                model_point.y = (model_view_transformed_translation->at(correspondence->first[0][i])).y;
                model_point.z = (model_view_transformed_translation->at(correspondence->first[0][i])).z;
                sampling_keypoints->push_back(model_point);

                pcl::PointXYZ scene_point;
                scene_point.x = (target_view->at(correspondence->first[1][i])).x;
                scene_point.y = (target_view->at(correspondence->first[1][i])).y;
                scene_point.z = (target_view->at(correspondence->first[1][i])).z;
                match_keypoints->push_back(scene_point);

                //  We are drawing a line for each pair of clustered correspondences found between the model and the scene
                viewer.addLine<pcl::PointXYZ, pcl::PointXYZ> (model_point, scene_point, 0, 255, 0, ss_line.str (), vp_2);

            }
            //show keypoints
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> scene_keypoints_color_handler (match_keypoints, 0, 0, 255);
            viewer.addPointCloud (match_keypoints, scene_keypoints_color_handler, "scene_keypoints",vp_2);
            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "scene_keypoints",vp_2);

            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> off_scene_model_keypoints_color_handler (sampling_keypoints, 0, 0, 255);
            viewer.addPointCloud (sampling_keypoints, off_scene_model_keypoints_color_handler, "model_keypoints",vp_2);
            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "model_keypoints",vp_2);

            viewer.spin ();
        }
    }//namespace pclib
}//namespace td