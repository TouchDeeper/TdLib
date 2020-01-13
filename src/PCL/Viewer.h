//
// Created by wang on 19-9-11.
//

#ifndef TDLIB_VIEWER_H
#define TDLIB_VIEWER_H

#include <vector>
#include "common_typedef.h"
namespace td{
    namespace pclib{
        class Viewer {
        public:
            Viewer(std::string window_name, std::vector<int> window_size = std::vector<int>(2,0)){
                viewer_.setWindowName(window_name);
                if(!(window_size[0] == 0 || window_size[1] == 0))
                    viewer_.setSize(window_size[0],window_size[1]);
            }

        public:
            pcl::visualization::PCLVisualizer viewer_ ;
        public:
            void ShowCorrespondence(const std::vector<std::vector<int>>& correspondence, const PointNCloudPtr model_view, const PointNCloudPtr scene_view,
            const Eigen::Matrix4d transformations,ColorHandlerPointN target_view_color, ColorHandlerPointN source_view_color, std::vector<int> line_rgb,
            double point_size = 1,std::vector<int> background_rgb = std::vector<int>(3,0));
            void ClearView(){
                viewer_.removeAllPointClouds();
                viewer_.removeAllShapes();
                viewer_.removeAllCoordinateSystems();
            }
        };
    }

}



#endif //TDLIB_VIEWER_H
