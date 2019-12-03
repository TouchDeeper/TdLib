//
// Created by wang on 19-11-24.
//
#include "eigen_tools.h"
namespace td{
#define MAXBUFSIZE  ((int) 1e6)

    void readMatrix(const std::string txt_path, Eigen::MatrixXi &result)
    {
        int cols = 0, rows = 0;
        int buff[MAXBUFSIZE];

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
        result = Eigen::MatrixXi::Zero(rows,cols);
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++)
                result(i,j) = buff[ cols*i+j ];

    }

}
