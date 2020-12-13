//
// Created by wang on 20-1-13.
//

#include "output.hpp"
namespace td{
    void saveTrajectoryTUM(const std::string &file_name, const VecMat4 &v_Twc)
    {
        std::ofstream f;
        f.open(file_name.c_str());
        f << std::fixed;

        for(size_t i = 0; i < v_Twc.size(); i++)
        {
            const Eigen::Matrix4d Twc = v_Twc[i];
            const Eigen::Vector3d t = Twc.block(0, 3, 3, 1);
            const Eigen::Matrix3d R = Twc.block(0, 0, 3, 3);
            const Eigen::Quaterniond q = Eigen::Quaterniond(R);

            f << std::setprecision(6) << i << " "
              << std::setprecision(9) << t[0] << " " << t[1] << " " << t[2] << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
        }
        f.close();
        std::cout << "save TUM traj to " << file_name << " done." << std::endl;
    }

}