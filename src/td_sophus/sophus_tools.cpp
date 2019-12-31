//
// Created by wang on 19-12-30.
//

#include "sophus_tools.h"
namespace td{
    /**
     * Calculation of biinvariant means.
     * @param Ts vector for storing T
     * @param so3 average so3
     * @param trans average translation
     */
    void LieAverage(const VecSE3d &Ts, Eigen::Vector3d &so3, Eigen::Vector3d &trans){
        VecSO3d Rs;
        trans = Eigen::Vector3d::Zero();
        for (const auto &T : Ts) {
            Eigen::Matrix3d R_ei = T.rotationMatrix();
            Sophus::SO3d R_temp(R_ei);
            Rs.push_back(R_temp);
            trans += T.translation();
        }
//        auto w = Sophus::average<std::vector<Sophus::SO3d, Eigen::aligned_allocator<Sophus::SO3d>>>(Rs);
        auto var = Sophus::average<VecSO3d>(Rs);
        so3 = var->log();
        trans /= Rs.size();
    }
}