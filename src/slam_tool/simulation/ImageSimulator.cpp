//
// Created by wang on 20-1-12.
//

#include "ImageSimulator.h"
namespace td{
    void ImageSimulator::createLandmarks()
    {
        float scale = 5;
        const double k = 0.5;

        landmarks_.clear();

        std::mt19937 gen{12345};
        std::normal_distribution<double> d_x{0.0, 4.0};
        std::normal_distribution<double> d_y{0.0, 10.0};
        std::normal_distribution<double> d_z{0.0, 10.0};
        for (int i = 0; i < 200; i ++)
        {
            Eigen::Vector3d pt;
            pt[0] = std::round(d_x(gen));
            pt[1] = std::round(d_y(gen));
            pt[2] = std::round(d_z(gen));
            landmarks_.push_back(pt);
        }
    }
    void ImageSimulator::createCameraPose(VecMat4 &v_Twc, const Eigen::Vector3d &point_focus)
    {
        float x_offset = 20;
        float y_offset = 0;
        float z_offset = -5;
        float scale = 10;
        static const Eigen::Vector3d b_cam_z(0, 0, 1);
        static const Eigen::Matrix3d R_w_base = (Eigen::Matrix3d() << 0, 0, -1,
                1, 0, 0,
                0, -1, 0).finished();// row major
        // std::cout << (R_w_base * Eigen::Vector3d(0,0,1)).transpose() << std::endl;

        v_Twc.clear();
        for (float angle = 0; angle < 4*360; angle+=15)
        {
            float theta = angle * 3.14159 / 180.0f;
            Eigen::Vector3d pt;
            pt[0] = 0.5 * cos(theta);
            pt[1] = sin(theta);
            pt[2] = theta / 20;

            pt = scale * pt;
            pt[0] += x_offset;
            pt[1] += y_offset;
            pt[2] += z_offset;

            Eigen::Vector3d b_cam_z_cur = R_w_base.transpose() * (point_focus - pt);
            Eigen::Matrix3d R_cur_base(Eigen::Quaterniond::FromTwoVectors(b_cam_z_cur, b_cam_z));
            // std::cout << pt.transpose() << ", " << (R_cur_base * b_cam_z_cur).transpose() << std::endl;
            Eigen::Matrix3d Rwc(R_w_base * R_cur_base.transpose());

            Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
            Twc.block(0, 0, 3, 3) = Rwc;
            Twc.block(0, 3, 3, 1) = pt;
            v_Twc.push_back(Twc);
        }
    }
    void ImageSimulator::detectFeatures(const Eigen::Matrix4d &Twc, VecVec2i &features, bool add_noise) {
        std::mt19937 gen{12345};
        const float pixel_sigma = 1.0;
        std::normal_distribution<> d{0.0, pixel_sigma};

        Eigen::Matrix3d Rwc = Twc.block(0, 0, 3, 3);
        Eigen::Vector3d twc = Twc.block(0, 3, 3, 1);
        Eigen::Matrix3d Rcw = Rwc.transpose();
        Eigen::Vector3d tcw = -Rcw * twc;

        features.clear();
        for (size_t l = 0; l < landmarks_.size(); ++l)
        {
            Eigen::Vector3d wP = landmarks_[l];
            Eigen::Vector3d cP = Rcw * wP + tcw;

            if(cP[2] < 0) continue;

            float noise_u = add_noise ? std::round(d(gen)) : 0.0f;
            float noise_v = add_noise ? std::round(d(gen)) : 0.0f;

            Eigen::Vector3d ft = K_ * cP;
            int u = ft[0]/ft[2] + 0.5 + noise_u;
            int v = ft[1]/ft[2] + 0.5 + noise_v;
            Eigen::Vector2i obs(u, v);
            features.push_back(obs);
            //std::cout << l << " " << obs.transpose() << std::endl;
        }
    }
}