//
// Created by wang on 19-4-29.
//

#ifndef TDLIB_KEYPOINT_H
#define TDLIB_KEYPOINT_H

#include "common_typedef.h"
namespace td{
    namespace pclib{
        /**
         * \brief compute the ISS keypoints
         * @param model input cloud
         * @param model_keypoints ISS keypoints
         */
        void IssKeypoint(PointRGBACloudPtr model, PointRGBACloudPtr model_keypoints, bool border_estimation=true);
        void IssKeypoint(PointRGBCloudPtr model, PointRGBCloudPtr model_keypoints, bool border_estimation=true);
    }//namespace pclib
}//namespace td
#endif //TDLIB_KEYPOINT_H
