//
// Created by wang on 19-12-6.
//
#include "2d_slam_tool.h"
namespace td{
    //位姿-->转换矩阵
    Eigen::Matrix3d PoseToTrans(Eigen::Vector3d x)
    {
        Eigen::Matrix3d trans;
        trans << cos(x(2)),-sin(x(2)),x(0),
                sin(x(2)), cos(x(2)),x(1),
                0,         0,    1;

        return trans;
    }
//转换矩阵－－＞位姿
    Eigen::Vector3d TransToPose(Eigen::Matrix3d trans)
    {
        Eigen::Vector3d pose;
        pose(0) = trans(0,2);
        pose(1) = trans(1,2);
        pose(2) = atan2(trans(1,0),trans(0,0));

        return pose;
    }
}
