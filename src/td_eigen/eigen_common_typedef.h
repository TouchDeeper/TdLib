//
// Created by wang on 19-10-3.
//

#ifndef TDLIB_EIGEN_COMMON_TYPEDEF_H
#define TDLIB_EIGEN_COMMON_TYPEDEF_H

#include <vector>
#include <Eigen/Core>
namespace td{
    typedef std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > VectorMatrix4d;
    typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > VectorVector3d;
    typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > VectorVector2d;
}

#endif //TDLIB_EIGEN_COMMON_TYPEDEF_H
