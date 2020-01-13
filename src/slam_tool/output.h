//
// Created by wang on 20-1-13.
//

#ifndef TDLIB_OUTPUT_H
#define TDLIB_OUTPUT_H

#include "../td_eigen/eigen_common_typedef.h"
#include <fstream>
#include <Eigen/Geometry>
#include <iostream>
#include <iomanip>
namespace td{
    void saveTrajectoryTUM(const std::string &file_name, const VecMat4 &v_Twc);
}
#endif //TDLIB_OUTPUT_H
