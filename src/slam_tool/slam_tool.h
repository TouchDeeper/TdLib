//
// Created by wang on 19-8-14.
//

#ifndef TDLIB_SLAM_TOOL_H
#define TDLIB_SLAM_TOOL_H

#include "slam_typedef.h"

namespace td{
    /**
     * use ICP align two trajectory
     * @input_param trajectory_1
     * @input_param trajectory_2
     * @output_param R12
     * @output_param t12
     */
    void IcpTrajectoryAlign(VecVector3d trajectory_1, VecVector3d trajectory_2, Eigen::Matrix3d &R12, Eigen::Vector3d &t12);

}// namespace td

#endif //TDLIB_SLAM_TOOL_H