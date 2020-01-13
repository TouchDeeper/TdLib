//
// Created by wang on 19-8-14.
//

#ifndef TDLIB_SLAM_TYPEDEF_H
#define TDLIB_SLAM_TYPEDEF_H

#include <vector>
#include <Eigen/Core>
#include <sophus/se3.hpp>

namespace td{
    typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVector3d;
    typedef std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> VecSopSE3;
}
#endif //TDLIB_SLAM_TYPEDEF_H
