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
            void ShowCorrespondence(const std::pair<std::vector<std::vector<int>>,float>* correspondence, const PointNCloudPtr model_view, const PointNCloudPtr scene_view,
            const Eigen::Matrix4d transformations);
        };
    }

}



#endif //TDLIB_VIEWER_H
