//
// Created by wang on 19-12-6.
//

#ifndef TDLIB_2D_SLAM_TOOL_H
#define TDLIB_2D_SLAM_TOOL_H

#include <Eigen/Core>
namespace td{
    //位姿-->转换矩阵
    Eigen::Matrix3d PoseToTrans(Eigen::Vector3d x);
    //转换矩阵－－＞位姿
    Eigen::Vector3d TransToPose(Eigen::Matrix3d trans);
}
#endif //TDLIB_2D_SLAM_TOOL_H
