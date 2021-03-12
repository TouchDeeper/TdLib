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
    template<typename T>
    Eigen::Matrix<T, 4, 4>
    estimate3DRigidTransform(const std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1> > >& points1,
                             const std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1> > >& points2)
    {
        // compute centroids
        Eigen::Matrix<T, 3, 1> c1, c2;
        c1.setZero(); c2.setZero();

        for (size_t i = 0; i < points1.size(); ++i)
        {
            c1 += points1.at(i);
            c2 += points2.at(i);
        }

        c1 /= points1.size();
        c2 /= points1.size();

        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> X(3, points1.size());
        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Y(3, points1.size());
        for (size_t i = 0; i < points1.size(); ++i)
        {
            X.col(i) = points1.at(i) - c1;
            Y.col(i) = points2.at(i) - c2;
        }

        Eigen::Matrix<T, 3, 3> H = X * Y.transpose();

        Eigen::JacobiSVD< Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> > svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);

        Eigen::Matrix<T, 3, 3> U = svd.matrixU();
        Eigen::Matrix<T, 3, 3> V = svd.matrixV();
        if (U.determinant() * V.determinant() < 0.0)
        {
            V.col(2) *= -1.0;
        }

        Eigen::Matrix<T, 3, 3> R = V * U.transpose();
        Eigen::Matrix<T, 3, 1> t = c2 - R * c1;

        return homogeneousTransform(R, t);
    }

    template<typename T>
    Eigen::Matrix<T, 4, 4>
    estimate3DRigidSimilarityTransform(const std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1> > >& points1,
                                       const std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1> > >& points2)
    {
        // compute centroids
        Eigen::Matrix<T, 3, 1> c1, c2;
        c1.setZero(); c2.setZero();

        for (size_t i = 0; i < points1.size(); ++i)
        {
            c1 += points1.at(i);
            c2 += points2.at(i);
        }

        c1 /= points1.size();
        c2 /= points1.size();

        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> X(3, points1.size());
        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Y(3, points1.size());
        for (size_t i = 0; i < points1.size(); ++i)
        {
            X.col(i) = points1.at(i) - c1;
            Y.col(i) = points2.at(i) - c2;
        }

        Eigen::Matrix<T, 3, 3> H = X * Y.transpose();

        Eigen::JacobiSVD< Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> > svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);

        Eigen::Matrix<T, 3, 3> U = svd.matrixU();
        Eigen::Matrix<T, 3, 3> V = svd.matrixV();
        if (U.determinant() * V.determinant() < 0.0)
        {
            V.col(2) *= -1.0;
        }

        Eigen::Matrix<T, 3, 3> R = V * U.transpose();

        std::vector<Eigen::Matrix<T, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3, 1> > > rotatedPoints1(points1.size());
        for (size_t i = 0; i < points1.size(); ++i)
        {
            rotatedPoints1.at(i) = R * (points1.at(i) - c1);
        }

        double sum_ss = 0.0, sum_tt = 0.0;
        for (size_t i = 0; i < points1.size(); ++i)
        {
            sum_ss += (points1.at(i) - c1).squaredNorm();
            sum_tt += (points2.at(i) - c2).dot(rotatedPoints1.at(i));
        }

        double scale = sum_tt / sum_ss;

        Eigen::Matrix<T, 3, 3> sR = scale * R;
        Eigen::Matrix<T, 3, 1> t = c2 - sR * c1;

        return homogeneousTransform(sR, t);
    }

    bool moveEnough(Eigen::Matrix4f& last, Eigen::Matrix4f& now, float th ){
        Sophus::SE3f Tlast(last);
        Sophus::SE3f Tnow(now);
        Sophus::SE3f Tlast_now = Tlast.inverse() * Tnow;
        td::Vec6f se3_last_now = Tlast_now.log();
        return se3_last_now.norm()>th;
    }
}// namespace td