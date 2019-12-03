//
// Created by wang on 19-3-18.
//

#include "FileOperation.h"
#include <fstream>
#include <sstream>
namespace td{
    void
    loadFeatureModels (const boost::filesystem::path &base_dir, const std::string &extension,
                       std::vector<vfh_model> &models)
    {
        if (!boost::filesystem::exists (base_dir) && !boost::filesystem::is_directory (base_dir))
            return;

        for (boost::filesystem::directory_iterator it (base_dir); it != boost::filesystem::directory_iterator (); ++it)
        {
            if (boost::filesystem::is_directory (it->status ()))
            {
                std::stringstream ss;
                ss << it->path ();
//            pcl::console::print_highlight ("Loading %s (%lu models loaded so far).\n", ss.str ().c_str (), (unsigned long)models.size ());
                loadFeatureModels (it->path (), extension, models);
            }
            if (boost::filesystem::is_regular_file (it->status ()) && boost::filesystem::extension (it->path ()) == extension)
            {
                vfh_model m;
//            if (loadHist (base_dir / it->path ().filename (), m))
                models.push_back (m);
            }
        }
    }

    bool find_file(const boost::filesystem::path& dir_path, const boost::filesystem::path& file_name, boost::filesystem::path& path_found) {
        const boost::filesystem::recursive_directory_iterator end;
        const auto it = std::find_if(boost::filesystem::recursive_directory_iterator(dir_path), end,
                                     [&file_name](const boost::filesystem::directory_entry& e) {
                                         return e.path().filename() == file_name;
                                     });
        if (it == end) {
            return false;
        } else {
            path_found = it->path();
            return true;
        }
    }
}
