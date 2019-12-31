//
// Created by wang on 19-12-30.
//

#ifndef TDLIB_SOPHUS_TOOLS_H
#define TDLIB_SOPHUS_TOOLS_H

#include "sophus_typedef.h"
#include <sophus/average.hpp>
#include <sophus/se3.hpp>
namespace td{
    /**
 * Calculation of biinvariant means.
 * @param Ts vector for storing T
 * @param so3 average so3
 * @param trans average translation
 */
    void LieAverage(const VecSE3d &Ts, Eigen::Vector3d &so3, Eigen::Vector3d &trans);
}


#endif //TDLIB_SOPHUS_TOOLS_H
