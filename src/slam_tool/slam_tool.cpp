//
// Created by wang on 19-8-14.
//

#include "slam_tool.h"
#include <Eigen/Dense>
namespace td{
    /**
     * use ICP align two trajectory
     * @input_param trajectory_1
     * @input_param trajectory_2
     * @output_param R12
     * @output_param t12
     */
    void IcpTrajectoryAlign(VecVector3d trajectory_1, VecVector3d trajectory_2, Eigen::Matrix3d &R12, Eigen::Vector3d &t12)
    {
        //计算质心坐标
        Eigen::Vector3d sume(0,0,0);
        Eigen::Vector3d sumg(0,0,0);
        for(int i=0;i< trajectory_1.size();i++)
        {
            sume += trajectory_1[i];
            sumg += trajectory_2[i];
        }
        Eigen::Vector3d centroid_1 = sume/trajectory_1.size();
        Eigen::Vector3d centroid_2 = sumg/trajectory_2.size();
        VecVector3d q_1,q_2;// trajectory that minus the centroid;
        //计算去质心坐标
        for(int i=0;i< trajectory_1.size();i++)
        {
            Eigen::Vector3d temp(trajectory_1[i]-centroid_1);
            q_1.push_back(temp);
            temp = trajectory_2[i]-centroid_2;
            q_2.push_back(temp);
        }
        assert(trajectory_1.size() == q_1.size());
        Eigen::Matrix3d W;
        for(int i=0;i< trajectory_1.size();i++)
        {
            W += q_1[i]*q_2[i].transpose();
        }
        //svd分解求R
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(W, Eigen::ComputeThinU | Eigen::ComputeThinV );
        Eigen::Matrix3d V = svd.matrixV(), U = svd.matrixU();
        Eigen::Matrix3d S = U.inverse() * W * V.transpose().inverse();
        R12 = U*V.transpose();

        //通过R求t
        t12 = centroid_1-R12*centroid_2;
    }

}// namespace td