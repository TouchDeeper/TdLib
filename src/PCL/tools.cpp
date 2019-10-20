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
        /**
         * compute the cloud resolution
         * @param cloud input cloud
         * @return the cloud resolution
         */
        double computeCloudResolution (const PointCloudPtr &cloud)
        {
            double res = 0.0;
            int n_points = 0;
            int nres;
            std::vector<int> indices (2);
            std::vector<float> sqr_distances (2);
            pcl::search::KdTree<Point> tree;
            tree.setInputCloud (cloud);

            for (size_t i = 0; i < cloud->size (); ++i)
            {
                if (! std::isfinite ((*cloud)[i].x))
                {
                    continue;
                }
                //Considering the second neighbor since the first is the point itself.
                nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
                if (nres == 2)
                {
                    res += sqrt (sqr_distances[1]);
                    ++n_points;
                }
            }
            if (n_points != 0)
            {
                res /= n_points;
            }
            return res;
        }
        double computeCloudResolution (const PointRGBCloudPtr &cloud)
        {
            double res = 0.0;
            int n_points = 0;
            int nres;
            std::vector<int> indices (2);
            std::vector<float> sqr_distances (2);
            pcl::search::KdTree<PointRGB> tree;
            tree.setInputCloud (cloud);

            for (size_t i = 0; i < cloud->size (); ++i)
            {
                if (! std::isfinite ((*cloud)[i].x))
                {
                    continue;
                }
                //Considering the second neighbor since the first is the point itself.
                nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
                if (nres == 2)
                {
                    res += sqrt (sqr_distances[1]);
                    ++n_points;
                }
            }
            if (n_points != 0)
            {
                res /= n_points;
            }
            return res;
        }
        double computeCloudResolution (const PointRGBACloudPtr &cloud)
        {
            double res = 0.0;
            int n_points = 0;
            int nres;
            std::vector<int> indices (2);
            std::vector<float> sqr_distances (2);
            pcl::search::KdTree<PointRGBA> tree;
            tree.setInputCloud (cloud);

            for (size_t i = 0; i < cloud->size (); ++i)
            {
                if (! std::isfinite ((*cloud)[i].x))
                {
                    continue;
                }
                //Considering the second neighbor since the first is the point itself.
                nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
                if (nres == 2)
                {
                    res += sqrt (sqr_distances[1]);
                    ++n_points;
                }
            }
            if (n_points != 0)
            {
                res /= n_points;
            }
            return res;
        }

        /**
        * show the align result
        * @param model the model view
        * @param scene the scene view
        * @param transformation the transformation s_T_o
        */
        void ShowAlignResult(const PointNCloudPtr model, const PointNCloudPtr scene, const Eigen::Matrix4d transformation,std::string viewer_name, ColorHandlerPointN scene_view_color, ColorHandlerPointN model_view_color,std::vector<int> window_size,double point_size,std::vector<int> background_color){
            pcl::visualization::PCLVisualizer viewer (viewer_name);
            viewer.setBackgroundColor(background_color[0],background_color[1],background_color[2]);
            if(!(window_size[0] == 0 || window_size[1] == 0))
                viewer.setSize(window_size[0],window_size[1]);
            viewer.addText (viewer_name, 10, 10, 18, 1.0, 1.0, 1.0, "text1");
            viewer.addCoordinateSystem (0.03);
            scene_view_color.setInputCloud(scene);
//            viewer.addPointCloud<PointN>(scene,ColorHandlerPointN (scene, scene_view_color[0], scene_view_color[1], scene_view_color[2]),"scene_view");
            viewer.addPointCloud<PointN>(scene,scene_view_color,"scene_view");
            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "scene_view");
            std::cout<<"point size = "<<point_size<<std::endl;
            PointNCloudPtr model_view_transformed(new PointNCloud);
            pcl::transformPointCloud(*model, *model_view_transformed, transformation);
//            viewer.addPointCloud<PointN>(model_view_transformed,ColorHandlerPointN (model_view_transformed, model_view_color[0], model_view_color[1], model_view_color[2]),"model_view");
            model_view_color.setInputCloud(model_view_transformed);
            viewer.addPointCloud<PointN>(model_view_transformed,model_view_color,"model_view");
            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "model_view");

            viewer.spin();
            viewer.close();
        }
    }

}
