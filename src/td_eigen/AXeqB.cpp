//
// Created by wang on 19-11-3.
//

#include "AXeqB.h"
namespace td{
    void LdltSolve(const Eigen::MatrixXd &A,const  Eigen::VectorXd &b, Eigen::VectorXd &x) {
        x = A.ldlt().solve(b);
    }
    void ColPivHouseHolderQR(const Eigen::MatrixXd &A, const Eigen::VectorXd &b, Eigen::VectorXd &x) {
        x = A.colPivHouseholderQr().solve(b);
    }

}


