//
// Created by wang on 20-4-27.
//
#include <Eigen/Core>

// Returns the 3D cross product skew symmetric matrix of a_ given 3D vector
template<typename T>
Eigen::Matrix<T, 3, 3> skew(const Eigen::Matrix<T, 3, 1>& vec)
{
    return (Eigen::Matrix<T, 3, 3>() << T(0), -vec(2), vec(1),
            vec(2), T(0), -vec(0),
            -vec(1), vec(0), T(0)).finished();
}

