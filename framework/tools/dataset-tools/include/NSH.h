/*

 Copyright (c) 2023 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

*/

#ifndef FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_NSH_H_
#define FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_NSH_H_


#include <ParameterManager.h>
#include <ParameterComponent.h>
#include <Parameters.h>

#include <io/sensor/Sensor.h>
#include <io/sensor/LidarSensor.h>

#include "DatasetReader.h"

namespace slambench {

    namespace io {
        class NSHReader : public DatasetReader {
          
        public:
            std::string input;
            bool lidar = true;

            explicit NSHReader(std::string name) : DatasetReader(std::move(name)) {
                this->addParameter(TypedParameter<std::string>("i",
                        "input-directory", "path of the NSH dataset directory",
                        &this->input, nullptr));
                this->addParameter(TypedParameter<bool>("lidar", "lidar",
                        "set to true or false to specify if the LIDAR stream need to be include in the slam file.",
                        &this->lidar, nullptr));
            }


            SLAMFile *GenerateSLAMFile() override;
        };
    } // namespace io
} // namespace slambench

#endif /* FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_NSH_H_ */
