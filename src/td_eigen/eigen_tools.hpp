//
// Created by wang on 19-11-24.
//

#ifndef TDLIB_EIGEN_TOOLS_HPP
#define TDLIB_EIGEN_TOOLS_HPP
#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Dense>
namespace td{
#define MAXBUFSIZE  ((int) 1e6)
    /**
     *  read a matrix from txt file, the txt file should be the one use `<< matrix<<std::endl;` export method,
     *  that is has an empty line in the end.
     * @param txt_path
     * @param result
     */
    template <typename Derived>
    void readMatrix(const char *txt_path, Eigen::MatrixBase<Derived> &result){
        int cols = 0, rows = 0;
        double buff[MAXBUFSIZE];

        // Read numbers from file into buffer.
        std::ifstream infile;
        infile.open(txt_path);
        if(!infile){
            std::cout<<"can't open "<<txt_path<<std::endl;
            exit(-1);
        }
        while (! infile.eof())
        {
            std::string line;
            getline(infile, line);

            int temp_cols = 0;
            std::stringstream stream(line);
            while(! stream.eof())
                stream >> buff[cols*rows+temp_cols++];

            if (temp_cols == 0)
                continue;

            if (cols == 0)
                cols = temp_cols;

            rows++;
        }

        infile.close();

        rows--;

        // Populate matrix with numbers.
        result.derived().resize(rows,cols);
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++)
                result(i,j) = buff[ cols*i+j ];
    }
    template <typename T>
    static Eigen::Quaternion<T> quaternionAverage(const std::vector<Eigen::Quaternion<T>>& quaternions)
    {
        if (quaternions.empty())
        {
            std::cerr << "Error trying to calculate the average quaternion of an empty set!\n";
            return Eigen::Quaternion<T>::Identity();
        }

        // first build a 4x4 matrix which is the elementwise sum of the product of each quaternion with itself
        Eigen::Matrix<T, 4, 4> A = Eigen::Matrix<T, 4, 4>::Zero();

        for (int q=0; q<quaternions.size(); ++q){
            Eigen::Matrix<T, 4, 1> quat_vec = quaternions[q].coeffs();
            A = A.eval() + quat_vec * quat_vec.transpose();
        }


        // normalise with the number of quaternions
        A /= quaternions.size();

        // Compute the SVD of this 4x4 matrix
        Eigen::JacobiSVD<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

        Eigen::Matrix<T, Eigen::Dynamic, 1> singularValues = svd.singularValues();
        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> U = svd.matrixU();

        // find the eigen vector corresponding to the largest eigen value
        int largestEigenValueIndex;
        float largestEigenValue;
        bool first = true;

        for (size_t i=0; i<singularValues.rows(); ++i)
        {
            if (first)
            {
                largestEigenValue = singularValues(i);
                largestEigenValueIndex = i;
                first = false;
            }
            else if (singularValues(i) > largestEigenValue)
            {
                largestEigenValue = singularValues(i);
                largestEigenValueIndex = i;
            }
        }

        Eigen::Quaternion<T> average(U(3, largestEigenValueIndex), U(0, largestEigenValueIndex), U(1, largestEigenValueIndex), U(2, largestEigenValueIndex));

        return average;
    }
}
#endif //TDLIB_EIGEN_TOOLS_HPP
