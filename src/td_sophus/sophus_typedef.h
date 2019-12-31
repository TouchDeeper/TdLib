//
// Created by wang on 19-10-4.
//

#ifndef TDLIB_SOPHUS_TYPEDEF_H
#define TDLIB_SOPHUS_TYPEDEF_H

#include <vector>
//#include <sophus/se3.h>
#include <sophus/se3.hpp>



namespace td{
    typedef std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d> > VectorSE3;

    typedef std::vector<Sophus::SO3d, Eigen::aligned_allocator<Sophus::SO3d> > VecSO3;

    typedef std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d> > VecSE3d;
    typedef std::vector<Sophus::SO3d, Eigen::aligned_allocator<Sophus::SO3d> > VecSO3d;


}
#endif //TDLIB_SOPHUS_TYPEDEF_H
