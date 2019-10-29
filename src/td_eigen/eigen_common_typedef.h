//
// Created by wang on 19-10-3.
//

#ifndef TDLIB_EIGEN_COMMON_TYPEDEF_H
#define TDLIB_EIGEN_COMMON_TYPEDEF_H

#include <vector>
#include <Eigen/Core>
namespace td{
    typedef Eigen::Matrix<double,6,1> Vector6d;

    typedef std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > VectorMatrix4d;
    typedef std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > VectorMatrix4f;
    typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > VectorVector3d;
    typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > VectorVector2d;
    typedef std::vector<Vector6d, Eigen::aligned_allocator<Vector6d> > VectorVector6d;
    typedef std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> > VectorMatrix3d;
    typedef std::vector<std::pair<Eigen::Matrix4d, double>, Eigen::aligned_allocator<std::pair<Eigen::Matrix4d, double>> > VectorPairMatrix4dDouble;

}

#endif //TDLIB_EIGEN_COMMON_TYPEDEF_H
