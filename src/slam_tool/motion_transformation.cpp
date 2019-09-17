//
// Created by wang on 19-9-17.
//
#include "motion_transformation.h"
namespace td{
    Sophus::SE3 EulerTranslatetoSE3(std::vector<double> euler_translate){

        Eigen::Vector3d o1_euler_angle_on(euler_translate[3],euler_translate[4],euler_translate[5]);
        Eigen::Matrix3d o1_R_on = (Eigen::AngleAxisd(o1_euler_angle_on[0],Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(o1_euler_angle_on[1],Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(o1_euler_angle_on[2],Eigen::Vector3d::UnitX())).matrix();
        Sophus::SE3 o1_T_on_sop(o1_R_on,Eigen::Vector3d(o1_euler_angle_on[0],o1_euler_angle_on[1],o1_euler_angle_on[2]));
        return o1_T_on_sop;
    }

}//namespace td
