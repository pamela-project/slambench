/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_DARPASUBT_H_
#define FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_DARPASUBT_H_

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

    class DARPASubtReader : public DatasetReader {

    private:
            static constexpr image_params_t img_params_ori = { 720, 540, 10.0, 1.0};
            // original recitified image size: 720*540, make it become multiple of 16
            static constexpr image_params_t img_params_rect = { 720, 536, 10.0, 1.0};

            static constexpr CameraSensor::intrinsics_t cam0_intri_anymal1_rect
                    = { 348.12911695 / img_params_rect.width, 348.12911695 / img_params_rect.height,
                        353.85713196 / img_params_rect.width, 264.06343842 / img_params_rect.height };

            static constexpr CameraSensor::intrinsics_t cam1_intri_anymal1_rect
                    = { 348.12911695 / img_params_rect.width, 348.12911695 / img_params_rect.height,
                        353.85713196 / img_params_rect.width, 264.06343842 / img_params_rect.height };

    public:
        std::string input;
        bool grey=true, imu = false, gt = true, stereo = true, lidar = true;

        explicit DARPASubtReader(std::string name) : DatasetReader(std::move(name)) {
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

        void get_DARPSSubt_params(CameraSensor::intrinsics_t &cam_intrinsics_lgrey,
                            CameraSensor::intrinsics_t &cam_intrinsics_rgrey,
                            CameraSensor::distortion_type_t &distortion_type,
                            CameraSensor::distortion_coefficients_t &cam_distortion_lgrey,
                            CameraSensor::distortion_coefficients_t &cam_distortion_rgrey) {
                                
                if (input.find("anymal1") != std::string::npos && input.find("sync") != std::string::npos) {
                    
                    std::cout << "Loading params of rectified ANYMAL1 cam0 and cam1..." << std::endl;
                    for (uint32_t i = 0; i < 4; i++) {
                        cam_intrinsics_lgrey[i] = cam0_intri_anymal1_rect[i];
                        cam_intrinsics_rgrey[i] = cam0_intri_anymal1_rect[i];
                    }

                    distortion_type = CameraSensor::NoDistortion;
                    for (uint32_t i = 0; i < 5; i++) {
                        cam_distortion_lgrey[i] = 0.0;
                        cam_distortion_rgrey[i] = 0.0;
                    }

                }
            }
    

        SLAMFile* GenerateSLAMFile() override;
    };

  }  // namespace io
}  // namespace slambench

#endif /* FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_DARPASUBT_H_ */