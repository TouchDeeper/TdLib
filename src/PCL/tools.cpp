//
// Created by wang on 19-4-13.
//
#include "tools.h"

namespace td{
    namespace pclib{
        void estimate_normals (	PointCloudPtr cloud_xyz,
                                   PointNCloudPtr cloud_N,
                                   float vp_x,
                                   float vp_y,
                                   float vp_z,
                                   int k_neighbor){
            pcl::NormalEstimationOMP<Point, pcl::Normal> normal_estimator;

            normal_estimator.setKSearch (k_neighbor);

            normal_estimator.setViewPoint(vp_x, vp_y, vp_z);
            NormalCloudPtr normals (new NormalCloud);
            normal_estimator.setInputCloud (cloud_xyz);
            normal_estimator.compute (*normals);

            // Concatenate cloud_xyz and normals into cloud_N
            pcl::concatenateFields(*cloud_xyz, *normals, *cloud_N);
        }
        void estimate_normals (	PointCloudPtr cloud_xyz,
                                   PointNCloudPtr cloud_N,
                                   float vp_x,
                                   float vp_y,
                                   float vp_z,
                                   float search_radius){
            pcl::NormalEstimationOMP<Point, pcl::Normal> normal_estimator;

            normal_estimator.setRadiusSearch (search_radius);

            normal_estimator.setViewPoint(vp_x, vp_y, vp_z);
            NormalCloudPtr normals (new NormalCloud);
            normal_estimator.setInputCloud (cloud_xyz);
            normal_estimator.compute (*normals);

            // Concatenate cloud_xyz and normals into cloud_N
            pcl::concatenateFields(*cloud_xyz, *normals, *cloud_N);
        }
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
         * @param source the source view
         * @param target the target view
         * @param t_T_s the transformation t_T_s
         */
        void ShowAlignResult(const PointNCloudPtr source, const PointNCloudPtr target, const Eigen::Matrix4d t_T_s,std::string viewer_name,ColorHandlerPointN target_color, ColorHandlerPointN source_color,std::vector<int> window_size,double point_size,std::vector<int> background_color)
        {
            pcl::visualization::PCLVisualizer viewer (viewer_name);
            viewer.setBackgroundColor(background_color[0],background_color[1],background_color[2]);
            if(!(window_size[0] == 0 || window_size[1] == 0))
                viewer.setSize(window_size[0],window_size[1]);
            viewer.addText (viewer_name, 10, 10, 18, 1.0, 1.0, 1.0, "text1");
            viewer.addCoordinateSystem (0.03);
            target_color.setInputCloud(target);
//            viewer.addPointCloud<PointN>(target,ColorHandlerPointN (target, target_color[0], target_color[1], target_color[2]),"scene_view");
            viewer.addPointCloud<PointN>(target,target_color,"target_view");
            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "target_view");
            PointNCloudPtr source_transformed(new PointNCloud);
            pcl::transformPointCloud(*source, *source_transformed, t_T_s);
//            viewer.addPointCloud<PointN>(source_transformed,ColorHandlerPointN (source_transformed, model_view_color[0], model_view_color[1], model_view_color[2]),"model_view");
            source_color.setInputCloud(source_transformed);
            viewer.addPointCloud<PointN>(source_transformed,source_color,"source_view");
            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "source_view");

            viewer.spin();
            viewer.close();
        }
        /**
        * show the align result
        * @param source the source view
        * @param target the target view
        * @param t_T_s_after the transformation t_T_s
        */
        void ShowBeforeAndAfterAlign(const PointNCloudPtr source, const PointNCloudPtr target, const Eigen::Matrix4d t_T_s_before, const Eigen::Matrix4d t_T_s_after,std::string viewer_name,ColorHandlerPointN target_color, ColorHandlerPointN source_color,std::vector<int> window_size,double point_size,std::vector<int> background_color)
        {
            pcl::visualization::PCLVisualizer viewer (viewer_name);
            int vp_1;//two viewport, left for showing align result, right for showing the keypoint correspondence
            int vp_2;
            viewer.createViewPort (0.0, 0, 0.5, 1.0, vp_1);
            viewer.createViewPort (0.5, 0, 1.0, 1.0, vp_2);
            viewer.setBackgroundColor(background_color[0],background_color[1],background_color[2],vp_1);
            if(!(window_size[0] == 0 || window_size[1] == 0))
                viewer.setSize(window_size[0],window_size[1]);
            //viewport 1
            viewer.addText ("before align", 10, 10, 18, 1.0, 1.0, 1.0, "text1",vp_1);
            viewer.addCoordinateSystem (0.03);
            target_color.setInputCloud(target);
//            viewer.addPointCloud<PointN>(target,ColorHandlerPointN (target, target_color[0], target_color[1], target_color[2]),"scene_view");
            viewer.addPointCloud<PointN>(target,target_color,"target_view_before",vp_1);
            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "target_view_before",vp_1);
            PointNCloudPtr source_transformed(new PointNCloud);
            pcl::transformPointCloud(*source, *source_transformed, t_T_s_before);
//            viewer.addPointCloud<PointN>(source_transformed,ColorHandlerPointN (source_transformed, model_view_color[0], model_view_color[1], model_view_color[2]),"model_view");
            source_color.setInputCloud(source_transformed);
            viewer.addPointCloud<PointN>(source_transformed,source_color,"source_view_before", vp_1);
            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "source_view_before",vp_1);

            //viewport 2
            viewer.addText ("after align", 10, 10, 18, 1.0, 1.0, 1.0, "text2",vp_2);
            viewer.addCoordinateSystem (0.03);
            target_color.setInputCloud(target);
//            viewer.addPointCloud<PointN>(target,ColorHandlerPointN (target, target_color[0], target_color[1], target_color[2]),"scene_view");
            viewer.addPointCloud<PointN>(target,target_color,"target_view_after",vp_2);
            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "target_view_after",vp_2);
            pcl::transformPointCloud(*source, *source_transformed, t_T_s_after);
//            viewer.addPointCloud<PointN>(source_transformed,ColorHandlerPointN (source_transformed, model_view_color[0], model_view_color[1], model_view_color[2]),"model_view");
            source_color.setInputCloud(source_transformed);
            viewer.addPointCloud<PointN>(source_transformed,source_color,"source_view_after", vp_2);
            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "source_view_after",vp_2);

            viewer.spin();
            viewer.close();
        }
        /**
         * get infomation after the souce be registered to the target
         * @param input_source source cloud
         * @param target_cloud_kdtree_
         * @param inlier_squared_threshold squared distance threshold for inlier check
         * @param inlier_fraction inlier_size/input_source->size
         * @param inlier_size
         * @param fitness_score
         * @param tg_T_sr transformation from source to target
         */
        void getFitness(PointNCloudPtr input_source, pcl::search::KdTree<pcl::PointNormal> target_cloud_kdtree_,
                double inlier_squared_threshold,
                double& inlier_fraction, int& inlier_size,
                float& fitness_score, Eigen::Matrix4d tg_T_sr) {
            // Initialize variables
            std::vector<int> inliers;
            inliers.reserve (input_source->size ());
            fitness_score = 0.0f;

            // Transform the input dataset using the population transformation
            td::pclib::PointNCloudPtr source_transformed(new td::pclib::PointNCloud);
//    source_transformed->resize (target_cloud_->size ());
            pcl::transformPointCloud (*input_source, *source_transformed, tg_T_sr);

            // For each point in the source dataset
            for (size_t i = 0; i < source_transformed->points.size (); ++i)
            {
                // Find its nearest neighbor in the target
                std::vector<int> nn_indices (1);
                std::vector<float> nn_dists (1);
                target_cloud_kdtree_.nearestKSearch (source_transformed->points[i], 1, nn_indices, nn_dists);

                // Check if point is an inlier
                if (nn_dists[0] < inlier_squared_threshold)
                {
                    // Update inliers
                    inliers.push_back (static_cast<int> (i));

                    // Update fitness score
                    fitness_score += nn_dists[0];
                }
            }

            // Calculate MSE
            if (inliers.size () > 0)
                fitness_score /= static_cast<float> (inliers.size ());
            else
                fitness_score = std::numeric_limits<float>::max ();
            inlier_fraction = static_cast<float> (inliers.size ()) / static_cast<float> (input_source->size ());
            inlier_size = inliers.size();
        }
        /**
         * get outlier cloud after the souce be registered to the target
         * @param input_source source cloud
         * @param target_cloud_kdtree_
         * @param inlier_squared_threshold squared distance threshold for inlier check
         * @param tg_T_sr transformation from source to target
         * @return outlier_cloud ptr
         */
        PointNCloudPtr getOutlierCloud(PointNCloudPtr input_source, pcl::search::KdTree<pcl::PointNormal> target_cloud_kdtree_,
                                                  double inlier_squared_threshold,Eigen::Matrix4d tg_T_sr) {
            PointNCloudPtr outlier_cloud(new PointNCloud);
            // Initialize variables
            std::vector<int> inliers;
            inliers.reserve (input_source->size ());
//            fitness_score = 0.0f;

            // Transform the input dataset using the population transformation
            PointNCloudPtr source_transformed(new PointNCloud);
//    source_transformed->resize (target_cloud_->size ());
            pcl::transformPointCloud (*input_source, *source_transformed, tg_T_sr);

            // For each point in the source dataset
            for (size_t i = 0; i < source_transformed->points.size (); ++i)
            {
                // Find its nearest neighbor in the target
                std::vector<int> nn_indices (1);
                std::vector<float> nn_dists (1);
                target_cloud_kdtree_.nearestKSearch (source_transformed->points[i], 1, nn_indices, nn_dists);

                // Check if point is an inlier
                if (nn_dists[0] < inlier_squared_threshold)
                {
                    // Update inliers
                    inliers.push_back (static_cast<int> (i));

                    // Update fitness score
//                    fitness_score += nn_dists[0];
                } else
                    outlier_cloud->push_back(input_source->points[i]);

            }

            // Calculate MSE
//            if (inliers.size () > 0)
//                fitness_score /= static_cast<float> (inliers.size ());
//            else
//                fitness_score = std::numeric_limits<float>::max ();
//            inlier_fraction = static_cast<float> (inliers.size ()) / static_cast<float> (input_source->size ());
//            inlier_size = inliers.size();
            return outlier_cloud;

        }

    }

}
