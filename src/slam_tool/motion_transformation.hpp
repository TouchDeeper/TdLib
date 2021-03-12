//
// Created by wang on 19-9-17.
//

#ifndef TDLIB_MOTION_TRANSFORMATION_HPP
#define TDLIB_MOTION_TRANSFORMATION_HPP

#include <vector>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <Eigen/Dense>

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

    template<typename T>
    Eigen::Matrix<T, 3, 3> AngleAxisToRotationMatrix(const Eigen::Matrix<T, 3, 1>& rvec)
    {
        T angle = rvec.norm();
        if (angle == T(0))
        {
            return Eigen::Matrix<T, 3, 3>::Identity();
        }

        Eigen::Matrix<T, 3, 1> axis;
        axis = rvec.normalized();

        Eigen::Matrix<T, 3, 3> rmat;
        rmat = Eigen::AngleAxis<T>(angle, axis);

        return rmat;
    }
    template<typename T>
    Eigen::Quaternion<T> AngleAxisToQuaternion(const Eigen::Matrix<T, 3, 1>& rvec)
    {
        Eigen::Matrix<T, 3, 3> rmat = AngleAxisToRotationMatrix<T>(rvec);

        return Eigen::Quaternion<T>(rmat);
    }
    template<typename T>
    Eigen::Matrix<T, 3, 1> RotationToAngleAxis(const Eigen::Matrix<T, 3, 3> & rmat)
    {
        Eigen::AngleAxis<T> angleaxis;
        angleaxis.fromRotationMatrix(rmat);
        return angleaxis.angle() * angleaxis.axis();

    }
    template<typename T>
    void QuaternionToAngleAxis(const T* const q, Eigen::Matrix<T, 3, 1>& rvec)
    {
        Eigen::Quaternion<T> quat(q[3], q[0], q[1], q[2]);

        Eigen::Matrix<T, 3, 3> rmat = quat.toRotationMatrix();

        Eigen::AngleAxis<T> angleaxis;
        angleaxis.fromRotationMatrix(rmat);

        rvec = angleaxis.angle() * angleaxis.axis();
    }
    template<typename T>
    Eigen::Matrix<T,4,4> QuaternionMultMatLeft(const Eigen::Quaternion<T>& q)
    {
        return (Eigen::Matrix<T,4,4>() << q.w(), -q.z(), q.y(), q.x(),
                q.z(), q.w(), -q.x(), q.y(),
                -q.y(), q.x(), q.w(), q.z(),
                -q.x(), -q.y(), -q.z(), q.w()).finished();
    }

    template<typename T>
    Eigen::Matrix<T,4,4> QuaternionMultMatRight(const Eigen::Quaternion<T>& q)
    {
        return (Eigen::Matrix<T,4,4>() << q.w(), q.z(), -q.y(), q.x(),
                -q.z(), q.w(), q.x(), q.y(),
                q.y(), -q.x(), q.w(), q.z(),
                -q.x(), -q.y(), -q.z(), q.w()).finished();
    }
    /**
     * convert a small theta to corresponding deltaQ
     * @tparam Derived template
     * @param theta a small angle
     * @return corresponding deltaQ
     */
    template <typename Derived>
    Eigen::Quaternion<typename Derived::Scalar> deltaQ(const Eigen::MatrixBase<Derived> &theta)
    {
        typedef typename Derived::Scalar Scalar_t;

        Eigen::Quaternion<Scalar_t> dq;
        Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
        half_theta /= static_cast<Scalar_t>(2.0);
        dq.w() = static_cast<Scalar_t>(1.0);
        dq.x() = half_theta.x();
        dq.y() = half_theta.y();
        dq.z() = half_theta.z();
        dq.normalize();
        return dq;
    }

}// namespace td

#endif //TDLIB_MOTION_TRANSFORMATION_HPP
