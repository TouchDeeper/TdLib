/*
 * @Description: 读写文件管理
 */


#include <string>
#include <iostream>
#include <fstream>
#include <opencv2/core/persistence.hpp>
#include <Eigen/Core>
#include <cxeigen.hpp>

namespace td {
class FileManager{
  public:
    static bool CreateFile(std::ofstream& ofs, std::string file_path);
    static bool CreateFileCVYaml(cv::FileStorage& fs, const std::string& file_path);
//    template<typename _Tp, int _rows, int _cols, int _options, int _maxRows, int _maxCols>
//    static bool Eigen2CVYaml(cv::FileStorage& fs, const Eigen::Matrix<_Tp, _rows, _cols, _options, _maxRows, _maxCols>& src, const std::string& node);
    template<typename _Tp, int _rows, int _cols, int _options, int _maxRows, int _maxCols>
    static bool Eigen2CVYaml(cv::FileStorage& fs, const Eigen::Matrix<_Tp, _rows, _cols, _options, _maxRows, _maxCols>& src, const std::string& node)
    {
        cv::Mat cv_mat;
        cv::eigen2cv(src,cv_mat);
        fs<<node<<cv_mat;
        return true;
    }
    static bool ReadFile(std::ifstream& ifs, std::string file_path);
    static bool InitDirectory(std::string directory_path, std::string use_for);
    static bool CreateDirectory(std::string directory_path, std::string use_for);
    static bool CreateDirectory(std::string directory_path);
};
}

