//
// Created by wang on 19-10-4.
//

#ifndef TDLIB_SOPHUS_TYPEDEF_H
#define TDLIB_SOPHUS_TYPEDEF_H

#include <vector>
#include <sophus/se3.h>

namespace td{
    typedef std::vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3> > VectorSE3;

}
#endif //TDLIB_SOPHUS_TYPEDEF_H
