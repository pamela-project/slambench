/*

 Copyright (c) 2019 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_DATASET_UTILS_H_
#define FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_DATASET_UTILS_H_

#include <boost/filesystem.hpp>
namespace slambench {
    namespace io {

        /**
        * Check for files and folders in a given directory and return true iff all exist
        *
        * @param directory_name directory to check for requirements
        * @param required vector of names of required files / folders
        */
        inline bool checkRequirements(const std::string& directory_name,
                                      const std::vector<std::string>& requirements) {

            try {
                if (!boost::filesystem::exists(directory_name)) return false;
                for (auto const &requirement : requirements) {
                    auto fullpath = directory_name+"/"+requirement;
                    if (!boost::filesystem::exists(fullpath)) {
                        std::cout << "File not found:" << fullpath << std::endl;
                        return false;
                    }
                }
            } catch (boost::filesystem::filesystem_error &e) {
                std::cerr << "I/O Error with directory " << directory_name << std::endl;
                std::cerr << e.what() << std::endl;
                return false;
            }

            return true;
        }
    }  // namespace io
}  // namespace slambench

#endif  // FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_DATASET_UTILS_H_
