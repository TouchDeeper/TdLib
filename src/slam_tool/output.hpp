//
// Created by wang on 20-1-13.
//

#ifndef TDLIB_OUTPUT_HPP
#define TDLIB_OUTPUT_HPP

#include "../td_eigen/eigen_common_typedef.h"
#include <fstream>
#include <Eigen/Geometry>
#include <iostream>
#include <iomanip>
namespace td{
    void saveTrajectoryTUM(const std::string &file_name, const VecMat4 &v_Twc);
    void saveTrajectoryTUM(std::ofstream& f, const Eigen::Matrix4f Twc, int i)
    {
        f << std::fixed;

        const Eigen::Vector3f t = Twc.block(0, 3, 3, 1);
        const Eigen::Matrix3f R = Twc.block(0, 0, 3, 3);
        const Eigen::Quaternionf q = Eigen::Quaternionf(R);

        f << std::setprecision(6) << i << " "
          << std::setprecision(9) << t[0] << " " << t[1] << " " << t[2] << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
    }
    void saveTrajectoryTUM(std::ofstream& f, const Eigen::Matrix4d Twc, int i)
    {
        f << std::fixed;

        const Eigen::Vector3d t = Twc.block(0, 3, 3, 1);
        const Eigen::Matrix3d R = Twc.block(0, 0, 3, 3);
        const Eigen::Quaterniond q = Eigen::Quaterniond(R);

        f << std::setprecision(6) << i << " "
          << std::setprecision(9) << t[0] << " " << t[1] << " " << t[2] << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
    }
}
#endif //TDLIB_OUTPUT_HPP
