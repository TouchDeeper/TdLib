/*
 * @Description: 一些文件读写的方法
 */
#include "file_manager.hpp"

#include <boost/filesystem.hpp>
#include <cxeigen.hpp>
#include "glog/logging.h"


namespace td {
bool FileManager::CreateFile(std::ofstream& ofs, std::string file_path) {
    ofs.close();
    boost::filesystem::remove(file_path.c_str());

    ofs.open(file_path.c_str(), std::ios::out);
    if (!ofs) {
        std::cout << "无法生成文件: " << std::endl << file_path << std::endl << std::endl;
        return false;
    }

    return true;
}
bool FileManager::CreateFileCVYaml(cv::FileStorage& fs, const std::string& file_path) {
    fs.release();
    boost::filesystem::remove(file_path.c_str());
    fs = cv::FileStorage(file_path, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        std::cout << "无法生成文件: " << std::endl << file_path << std::endl << std::endl;
        return false;
    }

    return true;
}
bool FileManager::ReadFile(std::ifstream &ifs, std::string file_path) {
    ifs.open(file_path, std::ifstream::in);
    if(!ifs)
    {
        std::cout << "无法打开文件: " << std::endl << file_path << std::endl << std::endl;
        return false;
    }
    return false;
}
bool FileManager::InitDirectory(std::string directory_path, std::string use_for) {
    if (boost::filesystem::is_directory(directory_path)) {
        boost::filesystem::remove_all(directory_path);
    }

    return CreateDirectory(directory_path, use_for);
}

bool FileManager::CreateDirectory(std::string directory_path, std::string use_for) {
    if (!boost::filesystem::is_directory(directory_path)) {
        boost::filesystem::create_directory(directory_path);
    }

    if (!boost::filesystem::is_directory(directory_path)) {
        std::cout << "无法创建文件夹: " << std::endl << directory_path << std::endl << std::endl;
        return false;
    }

    std::cout << use_for << "存放地址：" << std::endl << directory_path << std::endl << std::endl;
    return true;
}

bool FileManager::CreateDirectory(std::string directory_path) {
    if (!boost::filesystem::is_directory(directory_path)) {
        boost::filesystem::create_directory(directory_path);
    }

    if (!boost::filesystem::is_directory(directory_path)) {
        std::cout << "无法创建文件夹: " << std::endl << directory_path << std::endl << std::endl;
        return false;
    }

    return true;
}

//template<typename _Tp, int _rows, int _cols, int _options, int _maxRows, int _maxCols>
//bool FileManager::Eigen2CVYaml(cv::FileStorage &fs,
//                               const Eigen::Matrix<_Tp, _rows, _cols, _options, _maxRows, _maxCols> &src, const std::string& node) {
//    cv::Mat cv_mat;
//    cv::eigen2cv(src,cv_mat);
//    fs<<node<<cv_mat;
//    return true;
//}

}
