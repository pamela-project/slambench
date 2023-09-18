/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_NEWER_H_
#define FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_NEWER_H_

#include <ParameterManager.h>
#include <ParameterComponent.h>
#include <Parameters.h>

#include <io/sensor/Sensor.h>
#include <io/sensor/CameraSensor.h>
#include <io/sensor/IMUSensor.h>
#include <io/sensor/LidarSensor.h>
#include <io/sensor/GroundTruthSensor.h>

#include "DatasetReader.h"

namespace slambench {

  namespace io {

    class NewerCollegeReader : public DatasetReader {

    public:
        std::string input;
        bool grey=true, imu = false, gt = true, stereo = true, lidar = true;

        explicit NewerCollegeReader(std::string name) : DatasetReader(std::move(name)) {
                this->addParameter(TypedParameter<std::string>("i",
                        "input-directory", "path of the KITTI dataset directory",
                        &this->input, nullptr));
                this->addParameter(TypedParameter<bool>("grey", "grey",
                        "set to true or false to specify if the GREY stream need to be include in the slam file.",
                       &this->grey, nullptr));
                this->addParameter(TypedParameter<bool>("lidar", "lidar",
                        "set to true or false to specify if the LIDAR stream need to be include in the slam file.",
                        &this->lidar, nullptr));
                this->addParameter(TypedParameter<bool>("imu", "imu",
                        "set to true or false to specify if the IMU stream need to be include in the slam file.",
                        &this->imu, nullptr));
                this->addParameter(TypedParameter<bool>("gt", "gt",
                        "set to true or false to specify if the GROUNDTRUTH POSE stream need to be include in the slam file.",
                        &this->gt, nullptr));
                this->addParameter(TypedParameter<bool>("sgrey", "stereo-grey",
                        "set to true or false to specify if the STEREO GREY "
                        "stream need to be include in the slam file.",
                        &this->stereo, nullptr));
        }
    

        SLAMFile* GenerateSLAMFile() override;
    };

  }  // namespace io
}  // namespace slambench

#endif /* FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_NEWER_H_ */