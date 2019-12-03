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
}
#endif //TDLIB_EIGEN_TOOLS_HPP
