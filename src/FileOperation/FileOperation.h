//
// Created by wang on 19-3-18.
//

#ifndef TDLIB_FILEOPERATION_H
#define TDLIB_FILEOPERATION_H
#include <boost/filesystem.hpp>

typedef int vfh_model; // the file type that you want to store
void loadFeatureModels (const boost::filesystem::path &base_dir, const std::string &extension,
                   std::vector<vfh_model> &models);
#endif //TDLIB_FILEOPERATION_H
