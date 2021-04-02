//
// Created by wang on 19-9-17.
//
#include <iostream>
#include "motion_transformation.hpp"
namespace td{
    Sophus::SE3d EulerTranslatetoSE3(const std::vector<double>& euler_translate){

        Eigen::Vector3d o1_euler_angle_on(euler_translate[3],euler_translate[4],euler_translate[5]);
        Eigen::Matrix3d o1_R_on = (Eigen::AngleAxisd(o1_euler_angle_on[0],Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(o1_euler_angle_on[1],Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(o1_euler_angle_on[2],Eigen::Vector3d::UnitX())).matrix();
        Sophus::SE3d o1_T_on_sop(o1_R_on,Eigen::Vector3d(euler_translate[0],euler_translate[1],euler_translate[2]));
        return o1_T_on_sop;
    }
    Sophus::SE3d EulerTranslatetoSE3(const Eigen::Vector3d& euler_angle, const Eigen::Vector3d& translate){
        Eigen::Matrix3d o1_R_on = (Eigen::AngleAxisd(euler_angle[0],Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(euler_angle[1],Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(euler_angle[2],Eigen::Vector3d::UnitX())).matrix();
        Sophus::SE3d o1_T_on_sop(o1_R_on,translate);
        return o1_T_on_sop;

    }

    Sophus::SE3d FixedPreTranslateThenEulertoSE3(const Eigen::Vector3d &euler_angle, const Eigen::Vector3d& pre_translate){
        Eigen::Matrix3d o1_R_on = (Eigen::AngleAxisd(euler_angle[2],Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(euler_angle[1],Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(euler_angle[0],Eigen::Vector3d::UnitX())).matrix();
        Eigen::Vector3d translate = o1_R_on * pre_translate;
        Sophus::SE3d o1_T_on_sop(o1_R_on,translate);
        return o1_T_on_sop;

    }
    Eigen::Quaterniond EulerToQuaterninon(const Eigen::Vector3d &euler_angle){
        Eigen::Matrix3d o1_R_on = (Eigen::AngleAxisd(euler_angle[0],Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(euler_angle[1],Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(euler_angle[2],Eigen::Vector3d::UnitX())).matrix();
        Eigen::Quaterniond q(o1_R_on);
        return q;
    }

    Eigen::Matrix3d EulerToRotation(const Eigen::Vector3d &euler_angle)
    {
        Eigen::Matrix3d o1_R_on = (Eigen::AngleAxisd(euler_angle[0],Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(euler_angle[1],Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(euler_angle[2],Eigen::Vector3d::UnitX())).matrix();
        return o1_R_on;
    }
    /**
     * convert rotation to euler angle using this method https://www.docin.com/p-1714666272.html
     * @param rotation
     * @return 2,1,0 order euler angle
     */
    Eigen::Vector3d RotationToEulerAngle(const Eigen::Matrix3d &rotation){
        double xt_x,xt_y,xt_z;
        if(rotation(2,0) < 1)
        {
            if(rotation(2,0)>-1)
            {
                xt_y = asin(-rotation(2,0));
                xt_z = atan2(rotation(1,0), rotation(0,0));
                xt_x = atan2(rotation(2,1), rotation(2,2));
            } else //R(2,0) = -1
            {
                xt_y = M_PI_2;
                xt_z = -atan2(rotation(1,2), rotation(2,2));
                xt_x = 0;
            }
        } else{ // R(2,0) = 1
            xt_y = -M_PI_2;
            xt_z = atan2(-rotation(1,2), rotation(2,2));
            xt_x = 0;
        }
//        double xitax = atan2(rotation(2,1),rotation(2,2));
//        double c2 = sqrt(rotation(0,0)*rotation(0,0) + rotation(1,0) * rotation(1,0));
//        double xitay = atan2(-rotation(2,0), c2);
//        double xitaz = atan2(sin(xitax) * rotation(0,2) - cos(xitax) * rotation(0,1),cos(xitax) * rotation(1,1) - sin(xitax) * rotation(1,2));
        Eigen::Vector3d euler_angle(xt_z,xt_y,xt_x);
        return euler_angle;
    }
    /**
    * convert rotation to euler angle used in vins
    * @param rotation
    * @return ZYX order euler angle
    */
    Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R)
    {
        Eigen::Vector3d n = R.col(0);
        Eigen::Vector3d o = R.col(1);
        Eigen::Vector3d a = R.col(2);

        Eigen::Vector3d ypr(3);
        double y = atan2(n(1), n(0));
        double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
        double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
        ypr(0) = y;
        ypr(1) = p;
        ypr(2) = r;

        return ypr / M_PI * 180.0;
    }


}//namespace td
