//
// Created by wang on 20-4-27.
//
#include <Eigen/Core>
Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d &q)
{
    Eigen::Matrix3d ans;
    ans << 0.0, -q(2), q(1),
            q(2), 0.0, -q(0),
            -q(1), q(0), 0.0;
    return ans;
}
