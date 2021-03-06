//
// Created by wang on 19-3-18.
//

#ifndef TDLIB_FILEOPERATION_H
#define TDLIB_FILEOPERATION_H
#include <boost/filesystem.hpp>
namespace td{
    typedef int vfh_model; // the file type that you want to store
    void loadFeatureModels (const boost::filesystem::path &base_dir, const std::string &extension,
                            std::vector<vfh_model> &models);
    /**
     * recursive find a specified file in the given path
     * @param dir_path given path
     * @param file_name
     * @param path_found the file path
     * @return find:true
     */
    bool find_file(const boost::filesystem::path& dir_path, const boost::filesystem::path& file_name, boost::filesystem::path& path_found);
}

#endif //TDLIB_FILEOPERATION_H
