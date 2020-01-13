//
// Created by wang on 19-10-3.
//

#ifndef TDLIB_EIGEN_COMMON_TYPEDEF_H
#define TDLIB_EIGEN_COMMON_TYPEDEF_H

//#include <Eigen/StdVector>
#include <vector>
#include <Eigen/Core>
#include <map>
namespace td{

    typedef std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > VecMat4;
    typedef std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > VecMat4f;
    typedef std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> > VecMat3;

    // double matricies
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatXX;
    typedef Eigen::Matrix<double, 10, 10> Mat1010;
    typedef Eigen::Matrix<double, 13, 13> Mat1313;
    typedef Eigen::Matrix<double, 8, 10> Mat810;
    typedef Eigen::Matrix<double, 8, 3> Mat83;
    typedef Eigen::Matrix<double, 6, 6> Mat66;
    typedef Eigen::Matrix<double, 5, 3> Mat53;
    typedef Eigen::Matrix<double, 4, 3> Mat43;
    typedef Eigen::Matrix<double, 4, 2> Mat42;
    typedef Eigen::Matrix<double, 3, 3> Mat33;
    typedef Eigen::Matrix<double, 2, 2> Mat22;
    typedef Eigen::Matrix<double, 2, 3> Mat23;
    typedef Eigen::Matrix<double, 8, 8> Mat88;
    typedef Eigen::Matrix<double, 7, 7> Mat77;
    typedef Eigen::Matrix<double, 4, 9> Mat49;
    typedef Eigen::Matrix<double, 8, 9> Mat89;
    typedef Eigen::Matrix<double, 9, 4> Mat94;
    typedef Eigen::Matrix<double, 9, 8> Mat98;
    typedef Eigen::Matrix<double, 9, 9> Mat99;
    typedef Eigen::Matrix<double, 6, 6> Mat66;
    typedef Eigen::Matrix<double, 9, 6> Mat96;
    typedef Eigen::Matrix<double, 8, 1> Mat81;
    typedef Eigen::Matrix<double, 1, 8> Mat18;
    typedef Eigen::Matrix<double, 9, 1> Mat91;
    typedef Eigen::Matrix<double, 1, 9> Mat19;
    typedef Eigen::Matrix<double, 8, 4> Mat84;
    typedef Eigen::Matrix<double, 4, 8> Mat48;
    typedef Eigen::Matrix<double, 4, 4> Mat44;
    typedef Eigen::Matrix<double, 14, 14> Mat1414;
    typedef Eigen::Matrix<double, 15, 15> Mat1515;

// float matricies
    typedef Eigen::Matrix<float, 3, 3> Mat33f;
    typedef Eigen::Matrix<float, 10, 3> Mat103f;
    typedef Eigen::Matrix<float, 2, 2> Mat22f;
    typedef Eigen::Matrix<float, 3, 1> Vec3f;
    typedef Eigen::Matrix<float, 2, 1> Vec2f;
    typedef Eigen::Matrix<float, 6, 1> Vec6f;
    typedef Eigen::Matrix<float, 1, 8> Mat18f;
    typedef Eigen::Matrix<float, 6, 6> Mat66f;
    typedef Eigen::Matrix<float, 8, 8> Mat88f;
    typedef Eigen::Matrix<float, 8, 4> Mat84f;
    typedef Eigen::Matrix<float, 6, 6> Mat66f;
    typedef Eigen::Matrix<float, 4, 4> Mat44f;
    typedef Eigen::Matrix<float, 12, 12> Mat1212f;
    typedef Eigen::Matrix<float, 13, 13> Mat1313f;
    typedef Eigen::Matrix<float, 10, 10> Mat1010f;
    typedef Eigen::Matrix<float, 9, 9> Mat99f;
    typedef Eigen::Matrix<float, 4, 2> Mat42f;
    typedef Eigen::Matrix<float, 6, 2> Mat62f;
    typedef Eigen::Matrix<float, 1, 2> Mat12f;
    typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> MatXXf;
    typedef Eigen::Matrix<float, 14, 14> Mat1414f;

// double vectors
    typedef Eigen::Matrix<double, 15, 1> Vec15;
    typedef Eigen::Matrix<double, 14, 1> Vec14;
    typedef Eigen::Matrix<double, 13, 1> Vec13;
    typedef Eigen::Matrix<double, 10, 1> Vec10;
    typedef Eigen::Matrix<double, 9, 1> Vec9;
    typedef Eigen::Matrix<double, 8, 1> Vec8;
    typedef Eigen::Matrix<double, 7, 1> Vec7;
    typedef Eigen::Matrix<double, 6, 1> Vec6;
    typedef Eigen::Matrix<double, 5, 1> Vec5;
    typedef Eigen::Matrix<double, 4, 1> Vec4;
    typedef Eigen::Matrix<double, 3, 1> Vec3;
    typedef Eigen::Matrix<double, 2, 1> Vec2;
    typedef Eigen::Matrix<int, 2, 1> Vec2i;
    typedef Eigen::Matrix<double, 1, 1> Vec1;
    typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VecX;

// float vectors
    typedef Eigen::Matrix<float, 12, 1> Vec12f;
    typedef Eigen::Matrix<float, 8, 1> Vec8f;
    typedef Eigen::Matrix<float, 10, 1> Vec10f;
    typedef Eigen::Matrix<float, 4, 1> Vec4f;
    typedef Eigen::Matrix<float, 12, 1> Vec12f;
    typedef Eigen::Matrix<float, 13, 1> Vec13f;
    typedef Eigen::Matrix<float, 9, 1> Vec9f;
    typedef Eigen::Matrix<float, Eigen::Dynamic, 1> VecXf;
    typedef Eigen::Matrix<float, 14, 1> Vec14f;

// Quaternions
    typedef Eigen::Quaternion<double> Qd;
    typedef Eigen::Quaternion<float> Qf;

// Vector of Eigen vectors
    typedef std::vector<Vec2, Eigen::aligned_allocator<Vec2>> VecVec2;
    typedef std::vector<Vec2i, Eigen::aligned_allocator<Vec2i>> VecVec2i;
    typedef std::vector<Vec3, Eigen::aligned_allocator<Vec3>> VecVec3;
    typedef std::vector<Vec2f, Eigen::aligned_allocator<Vec2f>> VecVec2f;
    typedef std::vector<Vec3f, Eigen::aligned_allocator<Vec3f>> VecVec3f;
    typedef std::vector<Vec6, Eigen::aligned_allocator<Vec6> > VecVec6;
// Map of Eigen matrix
    typedef std::map<unsigned long, MatXX, std::less<unsigned long>, Eigen::aligned_allocator<MatXX>> MapMatXX;

}

#endif //TDLIB_EIGEN_COMMON_TYPEDEF_H
