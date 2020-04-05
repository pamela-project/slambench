/*

 Copyright (c) 2017-202 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 The interface with ROS datasets (rosbags) is supported by the RAIN Hub,
 which is funded by the Industrial Strategy Challenge Fund, part of the
 UK government’s modern Industrial Strategy. The fund is delivered by
 UK Research and Innovation and managed by EPSRC [EP/R026084/1].

 This code is licensed under the MIT License.

 */


#ifndef FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_TUM_ROS_H_
#define FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_TUM_ROS_H_


#include <ParameterManager.h>
#include <ParameterComponent.h>
#include <Parameters.h>

#include <io/sensor/Sensor.h>
#include <io/sensor/CameraSensor.h>
#include <io/sensor/DepthSensor.h>

#include "../../dataset-tools/include/DatasetReader.h"


namespace slambench {
    namespace io {
        class TUMROSReader : public DatasetReader {

        private :
            // these numbers were taken from TUM.h (originally from the TUM dataset paper)
            static constexpr CameraSensor::intrinsics_t fr1_intrinsics_rgb =
                    {0.80828125, 1.076041667, 0.4978125, 0.531875};
            static constexpr DepthSensor::intrinsics_t fr1_intrinsics_depth =
                    {0.92359375, 1.229375, 0.5171875, 0.4875};
            static constexpr CameraSensor::intrinsics_t fr2_intrinsics_rgb =
                    {0.81390624, 1.085416667, 0.5079687, 0.52020};
            static constexpr DepthSensor::intrinsics_t fr2_intrinsics_depth =
                    {0.9075, 1.212083333, 0.4825, 0.52708};

            // these numbers were taken from TUM.h (originally from ORBSLAM2 examples)
            static constexpr float fr1_fps = 30.0;
            static constexpr float fr1_bf = 40.0;
            static constexpr float fr1_ThDepth = 40.0;
            static constexpr float fr1_DepthMapFactor = 5000.0;

            static constexpr float fr2_fps = 30.0;
            static constexpr float fr2_bf = 40.0;
            static constexpr float fr2_ThDepth = 40.0;
            static constexpr float fr2_DepthMapFactor = 5000.0;  // Note: 5208.0 in TUM.h

            static constexpr CameraSensor::distortion_coefficients_t fr1_distortion_rgb =
                    {0.262383, -0.953104, -0.005358, 0.002628, 1.163314};
            static constexpr CameraSensor::distortion_coefficients_t fr2_distortion_rgb =
                    {0.231222, -0.784899, -0.003257, -0.000105, 0.917205};
            static constexpr DepthSensor::distortion_coefficients_t fr1_distortion_depth =
                    {0.262383, -0.953104, -0.005358, 0.002628, 1.163314};
            static constexpr DepthSensor::distortion_coefficients_t fr2_distortion_depth =
                    {0.231222, -0.784899, -0.003257, -0.000105, 0.917205};


        public :
            std::string input;
            bool grey = true, rgb = true, depth = true, gt = true, accelerometer = false;

            explicit TUMROSReader(std::string name) : DatasetReader(name) {
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


#endif /* FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_TUM_ROS_H_ */
