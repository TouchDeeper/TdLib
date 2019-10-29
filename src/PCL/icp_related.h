//
// Created by wang on 19-10-29.
//

#ifndef TDLIB_ICP_RELATED_H
#define TDLIB_ICP_RELATED_H

#include <Eigen/Core>
#include "common_typedef.h"
#include <pcl/registration/icp.h>
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
        bool Icp(Eigen::Matrix4d &guess_and_result, PointNCloudPtr source, PointNCloudPtr target,double cor_distance,int num_iteration, double transformation_epsilon, double euclidean_fitness_epsilon, int num_ransac, bool verbose);
    }
}
#endif //TDLIB_ICP_RELATED_H
