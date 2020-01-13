
//#include "TdLibrary/realsense.h"
//#include "TdLibrary/threadSafeStructure.h"
//#include "TdLibrary/random_tool.hpp"
#include "TdLibrary/loss_function.h"
#include "TdLibrary/eigen_tools.h"
#include "TdLibrary/tic_toc.h"
#include "TdLibrary/eigen_common_typedef.h"
#include <TdLibrary/slam_tool/motion_transformation.h>
//#include <thread>
//#include <atomic>
//#include "opencv2/core.hpp"
#include <iostream>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

using namespace std;
int main() {

//    td::hello();
////    td::rs::helloz();
//    td::threadsafe_queue<int> q1;
//
//    std::thread t2([&](){
//        std::cout<<"thread t2 is created"<<std::endl;
//        int j = 0;
//        int num = 100;
//        while(!q1.empty() || num > 0)
//        {
//            q1.wait_and_pop(j);
//            std::cout<<"thread t2 pop value "<<j<<std::endl;
//            num--;
//        }
//    });
//
//    for(int i = 0; i < 100; i++)
//        {
//            q1.push(i);
//            std::cout<<" main thread push value "<<i<<std::endl;
//        }
//    t2.join();
    td::slam::backend::CauchyLoss robust_function(1);

    float x = 1.123456789;
    float y = 0.000123456789;
    std::cout<<x<<std::endl<<y<<std::endl;
    int RUN_NUM = 5;
    cout<<pow(2, (RUN_NUM-1)/2)<<endl;

    Eigen::Matrix2d ma1;
    Eigen::Matrix<double,3,2> J;
    J << 1000,2,3,1,4,7;
    Eigen::Vector3d f;
    f<< 2,7,4;
//    Eigen::Matrix3d H = J.transpose()*J;
    Eigen::Matrix3d H;
    Eigen::Matrix3d H2;
    Eigen::Vector3d b;
//    Eigen::Vector3d b  = - J.transpose() * f;

    H << 19.5033, 24.5025,  32.835,
            24.5025,  32.835,    49.5,
            32.835,    49.5,     100;
    H2 << 19.5033, 24.5025,  32.835,
            24.5025,  32.835,    49.5,
            32.835,    49.5,     100;

    b << 827.953,1020.62,1373.91;
    auto diag1 = H.diagonal();
    auto D = diag1.array().sqrt();
    std::cout<<D<<std::endl;
//    H += diag1.asDiagonal();
//    H(0,0) += 1;
//    diag1.asDiagonal().toDenseMatrix()(0,0) += 1;
//    diag1(0) += 1;
//    std::cout<<diag1(0)<<std::endl;
    std::cout<<H<<std::endl;
//    diag1 = diag1.array().sqrt();
//    diag1.array() += 1;
//    diag1 = diag1.array().inverse();
//    std::cout<<diag1.segment(0,2)<<std::endl;
//    Eigen::Vector3d delta_x = H.ldlt().solve(b);
//    std::cout<<"delta_x : "<<std::endl<<delta_x<<std::endl;
//
//    Eigen::Vector2d jacobi_scaling = (J.array().square()).colwise().sum();
//    jacobi_scaling.array() = jacobi_scaling.array().sqrt();
//    jacobi_scaling.array()+=1;
//    jacobi_scaling.array() = jacobi_scaling.array().inverse();
//    auto diag1 = jacobi_scaling.asDiagonal();
////    Eigen::Matrix2d H_scaled = diag1 * H * diag1;
//    Eigen::Matrix3d H_scaled;
////    Eigen::Vector2d b_scaled = diag1 * b;
//    Eigen::Vector3d b_scaled;
//    H_scaled << 0.664829, 0.672178, 0.551119,
//                0.672178, 0.724909,  0.66863,
//                0.551119,  0.66863, 0.826446;
//    std::cout<<"H_scaled:\n"<<H_scaled<<std::endl;
//    b_scaled << 152.864,151.647,124.901;
//    Eigen::Vector3d scaled_vec;
//    scaled_vec << 0.184629, 0.148584, 0.0909091;
//    auto diag2 = scaled_vec.asDiagonal();
//    Eigen::Vector3d delta_x_scaled = H_scaled.ldlt().solve(b_scaled);
//    Eigen::Vector3d delta_x_2 = diag2 * delta_x_scaled;
//    std::cout<<"delta_x2 = "<<std::endl<<delta_x_2<<std::endl;
    Eigen::Quaterniond q1(0.35, 0.2, 0.3, 0.1);
    q1.normalize();
    Eigen::Matrix3d o1_R_o2 = q1.toRotationMatrix();
    std::cout<<"o1_R_o2:"<<o1_R_o2<<std::endl;
    Eigen::Vector3d o1_euler_angle_o2_ = o1_R_o2.eulerAngles(2,1,0);
    Eigen::Matrix3d o1_R_on = (Eigen::AngleAxisd(o1_euler_angle_o2_[0],Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(o1_euler_angle_o2_[1],Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(o1_euler_angle_o2_[2],Eigen::Vector3d::UnitX())).matrix();
    std::cout<<"o1_R_on:"<<o1_R_on<<std::endl;

//    Eigen::MatrixXd xMatrix;
//    xMatrix.resize(9,9);
//    xMatrix.setZero();
//    xMatrix.block(0,0,1,3) = Eigen::Vector3d(1,2,3);
//    std::cout<<"xMatrix = \n"<<xMatrix<<std::endl;
    Eigen::Matrix4d w_T_c;
    w_T_c <<   0.990688, -0.080013,  0.110158, -0.114349,
    -0.134897,  -0.68636,  0.714641, -0.759342,
    0.0184273, -0.722847, -0.690762,   1.02049,
    0,         0,         0,         1;
    Eigen::Matrix4d sim_w_T_c;
    sim_w_T_c <<  0.987156,  -0.094134,    0.12908,  -0.137189,
    0.158443,   0.680354,  -0.715552,   0.762191,
    -0.0204621,   0.726813,    0.68653,   -1.01583,
    0,          0,          0,          1;
    
    Eigen::Matrix4d calib = w_T_c * sim_w_T_c.inverse();
    std::cout<<"calib = \n"<<calib<<std::endl;
//    Eigen::MatrixXi read_matrix;
//    td::TicToc read_timer;
//    td::readMatrix("../relative_matrix.txt", read_matrix);
//    std::cout<<"read take "<<read_timer.toc()<<" ms"<<std::endl;
//    std::cout<<read_matrix<<std::endl;
    Eigen::Matrix4d m1;
    Eigen::Matrix4d m2;
    m1 << 0.990764, -0.0802061,   0.109333,  -0.113314,
    -0.134136,   -0.69777,   0.703651,  -0.743171,
    0.0198521,  -0.711817,  -0.702084,    1.03284,
    0,          0,          0,          1;
    m2 <<  0.975218, -0.120149,   0.18578, -0.209835,
            -0.216719, -0.687764,   0.69283, -0.731893,
            0.0445302, -0.715923, -0.696758,   1.02681,
            0,         0,         0,         1;
    Eigen::Matrix3d R1 = m1.block(0,0,3,3);
    Eigen::Matrix3d R2 = m2.block(0,0,3,3);
    Eigen::Vector3d t1 = m1.col(3).segment(0,3);
    Eigen::Vector3d t2 = m2.col(3).segment(0,3);
    Sophus::SE3d T1(R1, t1);
    Sophus::SE3d T2(R2, t2);
    td::Vector6d se1 = T1.log();
    td::Vector6d se2 = T2.log();
    std::cout<<se2.transpose()<<std::endl;
    if(m2.isApprox(m1,0.073)){
        std::cout<<"approx"<<std::endl;
    } else
        std::cout<<"no approx"<<std::endl;
    if(se2.isApprox(se1, 0.053))
        std::cout<<"approx"<<std::endl;
    else
        std::cout<<"no approx"<<std::endl;

//    Eigen::Matrix3d R;
//    R << 0.999566,  0.0245383, -0.0162906,
//    -0.024185,   0.999475,   0.021544,
//    0.0168107, -0.0211407,   0.999635;
//    Eigen::Vector3d euler = td::RotationToEulerAngle(R);
//    Eigen::Vector3d euler_eigen = R.eulerAngles(2,1,0);
//    std::cout<<"euler angle in eigen = "<<euler_eigen.transpose()<<std::endl;
//    std::cout<<"xt_z = "<<euler(0)<<std::endl;
//    std::cout<<"xt_y = "<<euler(1)<<std::endl;
//    std::cout<<"xt_x = "<<euler(2)<<std::endl;
//
//    Eigen::Matrix3d rotation = td::EulerToRotation(euler);
//    std::cout<<"rotation = \n"<<rotation<<std::endl;

    Eigen::Matrix4d T;
    T << -0.995503,  -0.073469,  0.0598016, 0.00815071,
        -0.0790468,   0.29634,  -0.951806,   0.341557,
        0.0522066,  -0.952253, -0.300815,   0.147992,
        0,          0,         0,           1;
    Eigen::Matrix3d R;
    R = T.block(0,0,3,3);
    Eigen::Vector3d t;
    t = T.col(3).segment(0,3);
    Sophus::SE3d SE3_T(R,t);
    td::Vector6d se3_T = SE3_T.log();
    std::cout<<se3_T <<std::endl;


}