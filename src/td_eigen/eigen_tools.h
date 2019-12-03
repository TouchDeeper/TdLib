//
// Created by wang on 19-11-24.
//

#ifndef TDLIB_EIGEN_TOOLS_H
#define TDLIB_EIGEN_TOOLS_H
#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Dense>
namespace td{
    /**
     *  read a matrix from txt file, the txt file should be the one use `<< matrix<<std::endl;` export method,
     *  that is has an empty line in the end.
     * @param txt_path
     * @param result
     */
    void readMatrix(const std::string txt_path, Eigen::MatrixXi &result);
}
#endif //TDLIB_EIGEN_TOOLS_H
