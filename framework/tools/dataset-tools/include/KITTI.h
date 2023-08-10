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
        enum DatasetOrigin { Default = 0, RD11_09_30, RD11_09_30_RECT};
        enum CameraID {LEFT_GREY, RIGHT_GREY, LEFT_RGB, RIGHT_RGB};

        private:
            static constexpr image_params_t img_params_unrect = { 1392, 512, 10.0, 1.0};
            // original recitified image size: 1226*370, make it become multiple of 16
            static constexpr image_params_t img_params_rect = { 1232, 368, 10.0, 1.0};
            
            // 2011_09_30 not rectified
            static constexpr CameraSensor::intrinsics_t intrinsics_110930_lgrey
                    = { 9.786977e+02 / img_params_unrect.width, 9.717435e+02 / img_params_unrect.height,
                        6.900000e+02 / img_params_unrect.width, 2.497222e+02 / img_params_unrect.height };
            
            static constexpr CameraSensor::intrinsics_t intrinsics_110930_rgrey
                    = { 9.892043e+02 / img_params_unrect.width, 9.832048e+02 / img_params_unrect.height,
                        7.020000e+02 / img_params_unrect.width, 2.616538e+02 / img_params_unrect.height };
            
            static constexpr CameraSensor::intrinsics_t intrinsics_110930_lrgb
                    = { 9.591977e+02 / img_params_unrect.width, 9.529324e+02 / img_params_unrect.height,
                        6.944383e+02 / img_params_unrect.width, 2.416793e+02 / img_params_unrect.height };
            
            static constexpr CameraSensor::intrinsics_t intrinsics_110930_rrgb
                    = { 9.035972e+02 / img_params_unrect.width, 8.979356e+02 / img_params_unrect.height,
                        6.979803e+02 / img_params_unrect.width, 2.392935e+02 / img_params_unrect.height };

            static constexpr CameraSensor::distortion_coefficients_t dc_110930_lgrey 
                    = {-3.792567e-01, 2.121203e-01, 9.182571e-04, 1.911304e-03, -7.605535e-02};

            static constexpr CameraSensor::distortion_coefficients_t dc_110930_rgrey
                    = {-3.720803e-01, 1.944116e-01, -1.077099e-04, -9.031379e-05, -6.314998e-02};
            
            static constexpr CameraSensor::distortion_coefficients_t dc_110930_lrgb
                    = {-3.725637e-01, 1.979803e-01, 1.799970e-04, 1.250593e-03, -6.608481e-02};

            static constexpr CameraSensor::distortion_coefficients_t dc_110930_rrgb
                    = {-3.726025e-01, 1.973869e-01, -5.746215e-04, 7.444947e-05, -6.699658e-02};

            // 2011_09_30_rect
            // divided by unresized width and height.
            static constexpr CameraSensor::intrinsics_t intrinsics_110930_rect
                    = { 7.113765e+02 / img_params_rect.width, 7.032691e+02 / img_params_rect.height,
                        6.048329e+02 / img_params_rect.width, 1.821206e+02 / img_params_rect.height };
        
        public:
            std::string input;
            bool grey = true, rgb = false, stereo = true, lidar = true, imu = false, gt = true;

            explicit KITTIReader(std::string name) : DatasetReader(std::move(name)) {
                this->addParameter(TypedParameter<std::string>("i",
                        "input-directory", "path of the KITTI dataset directory",
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
                this->addParameter(TypedParameter<bool>("sgrey", "stereo-grey",
                        "set to true or false to specify if the STEREO GREY "
                        "stream need to be include in the slam file.",
                        &this->stereo, nullptr));
            }

            static image_params_t get_image_params (bool rect) {
                if (rect) {
                    return img_params_rect;
                }
                return img_params_unrect;
            }

            DatasetOrigin check_data_origin() {

                if (input.find("2011_09_30") != std::string::npos && input.find("sync") == std::string::npos) {
                    return DatasetOrigin::RD11_09_30;
                } else if (input.find("2011_09_30") != std::string::npos && input.find("sync") != std::string::npos) {
                    return DatasetOrigin::RD11_09_30_RECT;
                }

                return DatasetOrigin::Default;
            }

            void get_params(CameraSensor::intrinsics_t &cam_intrinsics_lgrey,
                            CameraSensor::intrinsics_t &cam_intrinsics_rgrey,
                            CameraSensor::intrinsics_t &cam_intrinsics_lrgb,
                            CameraSensor::intrinsics_t &cam_intrinsics_rrgb,
                            CameraSensor::distortion_type_t &distortion_type,
                            CameraSensor::distortion_coefficients_t &cam_distortion_lgrey,
                            CameraSensor::distortion_coefficients_t &cam_distortion_rgrey,
                            CameraSensor::distortion_coefficients_t &cam_distortion_lrgb,
                            CameraSensor::distortion_coefficients_t &cam_distortion_rrgb) {
                if (input.find("2011_09_30") != std::string::npos && input.find("sync") == std::string::npos) {
                    
                    std::cout << "loading params of unrectified 2011_09_30 ..." << std::endl;
                    for (uint32_t i = 0; i < 4; i++) {
                        cam_intrinsics_lgrey[i] = intrinsics_110930_lgrey[i];
                        cam_intrinsics_rgrey[i] = intrinsics_110930_rgrey[i];
                        cam_intrinsics_lrgb[i] = intrinsics_110930_lrgb[i];
                        cam_intrinsics_rrgb[i] = intrinsics_110930_rrgb[i];
                    }

                    distortion_type = CameraSensor::RadialTangential;
                    for (uint32_t i = 0; i < 5; i++) {
                        cam_distortion_lgrey[i] = dc_110930_lgrey[i];
                        cam_distortion_rgrey[i] = dc_110930_rgrey[i];
                        cam_distortion_lrgb[i] = dc_110930_lrgb[i];
                        cam_distortion_rrgb[i] = dc_110930_rrgb[i];
                    }

                } else if (input.find("2011_09_30") != std::string::npos && input.find("sync") != std::string::npos) {
                    
                    std::cout << "loading params of rectified 2011_09_30 ..." << std::endl;
                    for (uint32_t i = 0; i < 4; i++) {
                        cam_intrinsics_lgrey[i] = intrinsics_110930_rect[i];
                        cam_intrinsics_rgrey[i] = intrinsics_110930_rect[i];
                        cam_intrinsics_lrgb[i] = intrinsics_110930_rect[i];
                        cam_intrinsics_rrgb[i] = intrinsics_110930_rect[i];
                    }

                    distortion_type = CameraSensor::NoDistortion;
                    for (uint32_t i = 0; i < 5; i++) {
                        cam_distortion_lgrey[i] = 0.0;
                        cam_distortion_rgrey[i] = 0.0;
                        cam_distortion_lrgb[i] = 0.0;
                        cam_distortion_rrgb[i] = 0.0;
                    }

                }
            }


            SLAMFile *GenerateSLAMFile() override;
        };
    } // namespace io
} // namespace slambench

#endif /* FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_KITTI_H_ */
