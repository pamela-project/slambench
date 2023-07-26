/*

 Copyright (c) 2023 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

*/

#ifndef FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_KITTI_H_
#define FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_KITTI_H_


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
        class KITTIReader : public DatasetReader {
        enum DatasetOrigin { Default = 0, RD11_09_30, RD11_10_03, RD11_09_30_CALIB, RD11_10_03_CALIB};
        private:
            static constexpr image_params_t img_params_uncalib = { 1392, 512, 10.0, 1.0};
            static constexpr image_params_t img_params_calib = { 1226, 370, 10.0, 1.0};

            static constexpr CameraSensor::intrinsics_t intrinsics_110930_calib
                    = { 707.09 / img_params_calib.width, 707.09 / img_params_calib.height,
                        601.89 / img_params_calib.width, 183.11 / img_params_calib.height };

            static const CameraSensor::distortion_type_t camera_distortion_type_calib
                    = CameraSensor::NoDistortion;
            
            static constexpr CameraSensor::distortion_coefficients_t distortion_110930_calib
                    = { 0.0, 0.0, 0.0, 0.0, 0.0 };
        
        public:
            std::string input;
            bool grey = true, rgb = true, lidar = true, imu = true, gt = true;

            explicit KITTIReader(std::string name) : DatasetReader(std::move(name)) {
                this->addParameter(TypedParameter<std::string>("i",
                        "input-directory", "path of the TUM dataset directory",
                        &this->input, nullptr));
                this->addParameter(TypedParameter<bool>("grey", "grey",
                        "set to true or false to specify if the GREY stream need to be include in the slam file.",
                        &this->grey, nullptr));
                this->addParameter(TypedParameter<bool>("rgb", "rgb",
                        "set to true or false to specify if the RGB stream need to be include in the slam file.",
                        &this->rgb, nullptr));
                this->addParameter(TypedParameter<bool>("lidar", "lidar",
                        "set to true or false to specify if the LIDAR stream need to be include in the slam file.",
                        &this->lidar, nullptr));
                this->addParameter(TypedParameter<bool>("imu", "imu",
                        "set to true or false to specify if the IMU stream need to be include in the slam file.",
                        &this->imu, nullptr));
                this->addParameter(TypedParameter<bool>("gt", "gt",
                        "set to true or false to specify if the GROUNDTRUTH POSE stream need to be include in the slam file.",
                        &this->gt, nullptr));
            }

            static image_params_t get_image_params (bool calib) {
                if (calib) {
                    return img_params_calib;
                }
                return img_params_uncalib;
            }

            DatasetOrigin get_sensor_params(CameraSensor::intrinsics_t & intrinsics,
                                            CameraSensor::distortion_type_t & distortion_type,
                                            CameraSensor::distortion_coefficients_t & distortion) {

                if (input.find("2011_09_30") != std::string::npos && input.find("sync") != std::string::npos) {
                    for (uint32_t i = 0; i < 4; i++) {
                        intrinsics[i] = intrinsics_110930_calib[i];
                    }
                    for (uint32_t i = 0; i < 5; i++) {
                        distortion[i] = distortion_110930_calib[i];
                    }
                    distortion_type = camera_distortion_type_calib;
                    return DatasetOrigin::RD11_09_30_CALIB;
                }

                return DatasetOrigin::Default;
            }

            SLAMFile *GenerateSLAMFile() override;
        };
    } // namespace io
} // namespace slambench

#endif /* FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_KITTI_H_ */
