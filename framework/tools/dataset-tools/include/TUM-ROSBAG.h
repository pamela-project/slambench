/*

 Copyright (c) 2017-2020 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 The development of the interface with ROS datasets (rosbags) is supported
 by the RAIN Hub, which is funded by the Industrial Strategy Challenge Fund,
 part of the UK government’s modern Industrial Strategy. The fund is
 delivered by UK Research and Innovation and managed by EPSRC [EP/R026084/1].

 This code is licensed under the MIT License.

 */


#ifndef FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_TUM_ROSBAG_H_
#define FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_TUM_ROSBAG_H_


#include <ParameterManager.h>
#include <ParameterComponent.h>
#include <Parameters.h>

#include <io/sensor/Sensor.h>
#include <io/sensor/CameraSensor.h>
#include <io/sensor/DepthSensor.h>

#include "TUM.h"


namespace slambench {
    namespace io {
        class TUMROSBAGReader : public TUMReader {

        public :
            explicit TUMROSBAGReader(std::string name) : TUMReader(name) {
                this->addParameter(TypedParameter<std::string>("i",
                        "input-directory", "path of the TUM dataset directory",
                        &this->input, nullptr));
                this->addParameter(TypedParameter<bool>("grey", "grey",
                        "set to true or false to specify if the GREY stream need to be include in the slam file.",
                        &this->grey, nullptr));
                this->addParameter(TypedParameter<bool>("rgb", "rgb",
                        "set to true or false to specify if the RGB stream need to be include in the slam file.",
                        &this->rgb, nullptr));
                this->addParameter(TypedParameter<bool>("depth", "depth",
                        "set to true or false to specify if the DEPTH stream need to be include in the slam file.",
                        &this->depth, nullptr));
                this->addParameter(TypedParameter<bool>("gt", "gt",
                        "set to true or false to specify if the GROUNDTRUTH POSE stream need to be include in the slam file.",
                        &this->gt, nullptr));
                this->addParameter(TypedParameter<bool>("acc", "accelerometer",
                        "set to true or false to specify if the ACCELEROMETER stream need to be include in the slam file.",
                        &this->accelerometer, nullptr));
            }

            SLAMFile *GenerateSLAMFile();
        };
    }
}


#endif /* FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_TUM_ROSBAG_H_ */
