//
// Created by wang on 19-10-29.
//
#include "icp_related.h"

namespace td{
    namespace pclib{
        /**
        * process the  ICP
        * @param guess_and_result object for storing the initial guess transformations and the result, if no converge, value will not change
        * @param source is the point cloud to be registered to the target.
        * @param target is target point cloud
         * @param cor_distance max correspondence distance threshold
         * @param euclidean_fitness_epsilon fitness change threshold, one of stop criteria
         * @param transformation_epsilon transformation change threshold. one  of stop criteria
         * @param verbose whether output each step's information
        */
        bool Icp(Eigen::Matrix4d &guess_and_result, PointNCloudPtr source, PointNCloudPtr target,double cor_distance,int num_iteration, double transformation_epsilon, double euclidean_fitness_epsilon, int num_ransac, bool verbose)
        {
            if(verbose)
            {
                pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal, double> icp;
                icp.setInputSource(source);
                icp.setInputTarget(target);
                icp.setMaxCorrespondenceDistance(cor_distance);
                icp.setTransformationEpsilon(transformation_epsilon);
                icp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon);
                icp.setRANSACIterations(num_ransac);
                icp.setRANSACOutlierRejectionThreshold(cor_distance);
                auto stop_criteria = icp.getConvergeCriteria();
                double last_fitness = 1000;
                Eigen::Matrix4d last_transformation;
                last_transformation.setIdentity();
                for (int i = 0; i < num_iteration; ++i) {
                    pcl::PointCloud<pcl::PointNormal> Final;
                    icp.setMaximumIterations(1);
                    icp.align(Final,guess_and_result);
                    guess_and_result = icp.getFinalTransformation();
                    Eigen::Matrix4d incremental_T = guess_and_result.inverse() * last_transformation;
                    last_transformation = guess_and_result;
                    double fitness = icp.getFitnessScore(cor_distance*cor_distance);
                    std::cout<<"iter "<<i<<"  fitness = "<<fitness<<"  incre_T.square_norm = "<<incremental_T.squaredNorm()-4<<std::endl;
                    double fitness_change = std::abs(last_fitness - fitness);
                    last_fitness = fitness;
                    if((incremental_T.squaredNorm() - 4)<transformation_epsilon || fitness_change<euclidean_fitness_epsilon)
                        break;
                }
                return true;
            } else
            {
                pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal, double> icp;
                icp.setInputSource(source);
                icp.setInputTarget(target);
                icp.setMaxCorrespondenceDistance(cor_distance);
                icp.setMaximumIterations(num_iteration);
                icp.setTransformationEpsilon(transformation_epsilon);
                icp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon);
                icp.setRANSACIterations(num_ransac);
                icp.setRANSACOutlierRejectionThreshold(cor_distance);
                pcl::PointCloud<pcl::PointNormal> Final;
                icp.align(Final, guess_and_result);
                if(icp.hasConverged())
                    guess_and_result = icp.getFinalTransformation();
                return icp.hasConverged();
            }



        }
    }
}