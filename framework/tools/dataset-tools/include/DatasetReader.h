/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#ifndef FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_DATASETREADER_H_
#define FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_DATASETREADER_H_

#include <ParameterComponent.h>

namespace slambench {
    namespace io {
        class SLAMFile;
    }
}

class DatasetReader : public ParameterComponent {
public :
    DatasetReader(const std::string& name) : ParameterComponent(name) {}
    virtual slambench::io::SLAMFile* GenerateSLAMFile() = 0;
};

#endif /* FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_DATASETREADER_H_ */
