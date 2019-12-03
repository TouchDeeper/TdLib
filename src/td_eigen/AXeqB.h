//
// Created by wang on 19-11-3.
//

#ifndef TDLIB_AXEQB_H
#define TDLIB_AXEQB_H
/**
 * Method              requirement                speed               speed   accuracy
 *                                               (small to medium)    (large)
 * ldlt                 positive or               +++                    +       ++
 *                  negative semidefinite
 * colPivHouseholerQR      None                     +                   -       +++
 */
#include <Eigen/Core>
#include <Eigen/Dense>
namespace td{
    /**
     * ldlt solve method
     * if A'x=b' can't satisy the requirement, A = A'(T)*A, b= A'(T)*b'
     * @param A
     * @param b
     * @param x
     */
    void LdltSolve(const Eigen::MatrixXd &A, const Eigen::VectorXd &b, Eigen::VectorXd &x);
    /**
     * colPivHouseholerQR method
     */
}   void ColPivHouseHolderQR(const Eigen::MatrixXd &A, const Eigen::VectorXd &b, Eigen::VectorXd &x);
#endif //TDLIB_AXEQB_H
