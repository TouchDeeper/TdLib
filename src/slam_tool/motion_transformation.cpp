//
// Created by wang on 19-9-17.
//
#include "motion_transformation.h"
namespace td{
    Sophus::SE3 EulerTranslatetoSE3(std::vector<double> euler_translate){

        Eigen::Vector3d o1_euler_angle_on(euler_translate[3],euler_translate[4],euler_translate[5]);
        Eigen::Matrix3d o1_R_on = (Eigen::AngleAxisd(o1_euler_angle_on[0],Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(o1_euler_angle_on[1],Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(o1_euler_angle_on[2],Eigen::Vector3d::UnitX())).matrix();
        Sophus::SE3 o1_T_on_sop(o1_R_on,Eigen::Vector3d(euler_translate[0],euler_translate[1],euler_translate[2]));
        return o1_T_on_sop;
    }
    Sophus::SE3 EulerTranslatetoSE3(Eigen::Vector3d euler_angle, Eigen::Vector3d translate){
        Eigen::Matrix3d o1_R_on = (Eigen::AngleAxisd(euler_angle[0],Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(euler_angle[1],Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(euler_angle[2],Eigen::Vector3d::UnitX())).matrix();
        Sophus::SE3 o1_T_on_sop(o1_R_on,translate);
        return o1_T_on_sop;

    }
    Eigen::Quaterniond EulerToQuaterninon(const Eigen::Vector3d &euler_angle){
        Eigen::Matrix3d o1_R_on = (Eigen::AngleAxisd(euler_angle[0],Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(euler_angle[1],Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(euler_angle[2],Eigen::Vector3d::UnitX())).matrix();
        Eigen::Quaterniond q(o1_R_on);
        return q;
    }
    /**
     * convert rotation to euler angle using this method https://www.docin.com/p-1714666272.html
     * @param rotation
     * @return 2,1,0 order euler angle
     */
    Eigen::Vector3d RotationToEulerAngle(const Eigen::Matrix3d &rotation){
        double xitax = atan2(rotation(2,1),rotation(2,2));
        double c2 = sqrt(rotation(0,0)*rotation(0,0) + rotation(1,0) * rotation(1,0));
        double xitay = atan2(-rotation(2,0), c2);
        double xitaz = atan2(sin(xitax) * rotation(0,2) - cos(xitax) * rotation(0,1),cos(xitax) * rotation(1,1) - sin(xitax) * rotation(1,2));
        Eigen::Vector3d euler_angle(xitaz,xitay,xitax);
        return euler_angle;
    }

}//namespace td
