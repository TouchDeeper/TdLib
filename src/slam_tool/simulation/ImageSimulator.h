//
// Created by wang on 20-1-12.
//

#ifndef TDLIB_IMAGESIMULATOR_H
#define TDLIB_IMAGESIMULATOR_H

#include "../../td_eigen/eigen_common_typedef.h"
#include <boost/shared_ptr.hpp>
#include <Eigen/Geometry>
namespace td{
    class ImageSimulator {
    public:
        ImageSimulator(){}
        void set_K(Eigen::Matrix3d K){
            K_ = K;
        }
        void createLandmarks();
        void createCameraPose(VecMat4 &v_Twc, const Eigen::Vector3d &point_focus);
        void detectFeatures(const Eigen::Matrix4d &Twc, VecVec2i &features, bool add_noise=true);
        void get_landmarks(VecVec3 &landmarks){
            landmarks = landmarks_;
        }
    private:
        Eigen::Matrix3d K_;
        VecVec3 landmarks_;
    public:
        typedef boost::shared_ptr< ::td::ImageSimulator> Ptr;
        typedef boost::shared_ptr< ::td::ImageSimulator const> ConstPtr;
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

}


#endif //TDLIB_IMAGESIMULATOR_H
