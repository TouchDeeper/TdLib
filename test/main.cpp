
//#include "TdLibrary/realsense.h"
//#include "TdLibrary/threadSafeStructure.h"
//#include "TdLibrary/random_tool.hpp"
#include "TdLibrary/loss_function.h"

//#include <thread>
//#include <atomic>
//#include "opencv2/core.hpp"
#include <iostream>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen3/Eigen/Dense>
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

}