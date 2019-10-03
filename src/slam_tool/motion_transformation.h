//
// Created by wang on 19-9-17.
//

#ifndef TDLIB_MOTION_TRANSFORMATION_H
#define TDLIB_MOTION_TRANSFORMATION_H

#include <vector>
#include "sophus/se3.h"
#include "sophus/so3.h"
namespace td{
    //Euler angle is ZYX convention
    Sophus::SE3 EulerTranslatetoSE3(std::vector<double> euler_translate);
    Sophus::SE3 EulerTranslatetoSE3(Eigen::Vector3d euler_angle, Eigen::Vector3d translate);
    Eigen::Quaterniond EulerToQuaterninon(const Eigen::Vector3d &euler_angle);
    Eigen::Vector3d RotationToEulerAngle(const Eigen::Matrix3d &rotation);

}// namespace td

#endif //TDLIB_MOTION_TRANSFORMATION_H
