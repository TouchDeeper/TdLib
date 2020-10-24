//
// Created by wang on 19-9-17.
//

#ifndef TDLIB_MOTION_TRANSFORMATION_H
#define TDLIB_MOTION_TRANSFORMATION_H

#include <vector>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
namespace td{
    //Euler angle is ZYX convention
    /**
     * euler translate to SE3d
     * @param euler_translate [translate, euler angle(ZYX)]
     * @return SE3d
     */
    Sophus::SE3d EulerTranslatetoSE3(const std::vector<double>& euler_translate);
    Sophus::SE3d EulerTranslatetoSE3(const Eigen::Vector3d& euler_angle, const Eigen::Vector3d& translate);
    /**
     * Pose convention of Gazebo, transformation by a fixed coordinate system, translate first then rotate,
     * refer to:https://answers.gazebosim.org//question/9578/pose-calculation-in-gazebosim-tutorial/

     * @param euler_angle fixed axis rpy(zyx) convention, euler anlge(xyz)
     * @param pre_translate
     * @return Sophus transformation
     */
    Sophus::SE3d FixedPreTranslateThenEulertoSE3(const Eigen::Vector3d &euler_angle, const Eigen::Vector3d& pre_translate);
    /**
     *
     * @param euler_angle(ZYX order)
     * @return
     */
    Eigen::Quaterniond EulerToQuaterninon(const Eigen::Vector3d &euler_angle);
    /**
     * compute the corresponding rotation matrix
     * @param euler_angle(ZYX order)
     * @return
     */
    Eigen::Matrix3d EulerToRotation(const Eigen::Vector3d &euler_angle);
    /**
    * convert rotation to euler angle using this method https://www.docin.com/p-1714666272.html
    * @param rotation
    * @return 2,1,0 order euler angle
    */
    Eigen::Vector3d RotationToEulerAngle(const Eigen::Matrix3d &rotation);

}// namespace td

#endif //TDLIB_MOTION_TRANSFORMATION_H
