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
    /**
     *
     * @param ofs
     * @param file_path
     * @param mode
        1:ios::app	追加模式。所有写入都追加到文件末尾。
        2:ios::ate	文件打开后定位到文件末尾。
        3:ios::out	打开文件用于写入。
        4:ios::trunc	如果该文件已经存在，其内容将在打开文件之前被截断，即把文件长度设为 0。
     * @return
     */
    static bool CreateFile(std::ofstream& ofs, std::string file_path, int mode);
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

