//
// Created by wang on 19-4-8.
//

#ifndef TDLIB_ENVELOPING_HPP
#define TDLIB_ENVELOPING_HPP

#include "common_typedef.h"

namespace td{
    namespace pclib{
        template <typename PointType>
        void BoundingBoxSize(const typename pcl::PointCloud<PointType>::Ptr cloud, Eigen::Vector3f &box, bool view_box){
            Eigen::Vector4f pcaCentroid;
            pcl::compute3DCentroid(*cloud, pcaCentroid);
            Eigen::Matrix3f covariance;
            pcl::computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
            Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
            Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();
            eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); //校正主方向间垂直
            eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
            eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));

            std::cout << "特征值va(3x1):\n" << eigenValuesPCA << std::endl;
            std::cout << "特征向量ve(3x3):\n" << eigenVectorsPCA << std::endl;
            std::cout << "质心点(4x1):\n" << pcaCentroid << std::endl;

            Eigen::Matrix4f T10 = Eigen::Matrix4f::Identity();
            Eigen::Matrix4f T01 = Eigen::Matrix4f::Identity();
            //T10,即0相对于1
            T10.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();   //R(-1).
            T10.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose()) *(pcaCentroid.head<3>());//  -R*t
            T01 = T10.inverse();

            std::cout << "变换矩阵T10(4x4):\n" << T10 << std::endl;
            std::cout << "逆变矩阵T01'(4x4):\n" << T01 << std::endl;

            typename pcl::PointCloud<PointType>::Ptr transformedCloud(new pcl::PointCloud<PointType>);
            pcl::transformPointCloud(*cloud, *transformedCloud, T10);
            PointType min_p1, max_p1;   //点云的最大值与最小值点
            Eigen::Vector3f c1, c;
            pcl::getMinMax3D(*transformedCloud, min_p1, max_p1);
            c1 = 0.5f*(min_p1.getVector3fMap() + max_p1.getVector3fMap());

            std::cout << "型心c1(3x1):\n" << c1 << std::endl;

            Eigen::Affine3f T01_aff(T01);
            pcl::transformPoint(c1, c, T01_aff);//c是0坐标系下的型心

            Eigen::Vector3f whd;
            box = max_p1.getVector3fMap() - min_p1.getVector3fMap();
            whd = box;
            float sc1 = (box(0) + box(1) + box(2)) / 3;  //点云平均尺度，用于设置主方向箭头大小

            std::cout << "width1=" << box(0) << endl;
            std::cout << "heght1=" << box(1) << endl;
            std::cout << "depth1=" << box(2) << endl;
            std::cout << "scale1=" << sc1 << endl;

            const Eigen::Quaternionf bboxQ1(Eigen::Quaternionf::Identity());
            const Eigen::Vector3f    bboxT1(c1);

            const Eigen::Quaternionf bboxQ(T01.block<3, 3>(0, 0));
            const Eigen::Vector3f    bboxT(c);


            //变换到原点的点云主方向
//            PointType op;
//            op.x = 0.0;
//            op.y = 0.0;
//            op.z = 0.0;
//            Eigen::Vector3f px, py, pz;
//            Eigen::Affine3f T10_aff(T10);
//            pcl::transformVector(eigenVectorsPCA.col(0), px, T10_aff);
//            pcl::transformVector(eigenVectorsPCA.col(1), py, T10_aff);
//            pcl::transformVector(eigenVectorsPCA.col(2), pz, T10_aff);
//            PointType pcaX;
//            pcaX.x = sc1 * px(0);
//            pcaX.y = sc1 * px(1);
//            pcaX.z = sc1 * px(2);
//            PointType pcaY;
//            pcaY.x = sc1 * py(0);
//            pcaY.y = sc1 * py(1);
//            pcaY.z = sc1 * py(2);
//            PointType pcaZ;
//            pcaZ.x = sc1 * pz(0);
//            pcaZ.y = sc1 * pz(1);
//            pcaZ.z = sc1 * pz(2);

            //visualization
            if(view_box){
                pcl::visualization::PCLVisualizer viewer;


                pcl::visualization::PointCloudColorHandlerCustom<PointType> color_handler(cloud, 255, 255, 0); //输入的初始点云相关
                viewer.addPointCloud(cloud, color_handler, "cloud");
                viewer.addCube(bboxT, bboxQ, whd(0), whd(1), whd(2), "bbox");
                viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,         pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "bbox");
                viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "bbox");


                viewer.addCoordinateSystem(0.5f*sc1);
                viewer.setBackgroundColor(0.0, 0.0, 0.0);
                while (!viewer.wasStopped())
                {
                    viewer.spinOnce();
                }
            }

        }
    }//namespace pclib
} //namespace td
#endif //TDLIB_ENVELOPING_HPP
