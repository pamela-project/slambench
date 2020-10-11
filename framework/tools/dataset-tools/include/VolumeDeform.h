/*

 Copyright (c) 2020 University of Edinburgh, Imperial College, University of
 Manchester. Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_VOLUMEDEFORM_H_
#define FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_VOLUMEDEFORM_H_

#include <ParameterComponent.h>
#include <ParameterManager.h>
#include <Parameters.h>
#include <io/sensor/CameraSensor.h>
#include <io/sensor/DepthSensor.h>
#include <io/sensor/GroundTruthSensor.h>
#include <io/sensor/Sensor.h>

#include "DatasetReader.h"

namespace slambench {

    namespace io {

        class VolumeDeformReader : public DatasetReader {
        private:
            static constexpr image_params_t image_params =
                    { 640, 480, 30.0, 1000.0 };
//            fx,fy,cx,cy           same for rgb and depth
            static constexpr CameraSensor::intrinsics_t intrinsics = {570.0 / image_params.width, 570.0 / image_params.height, 320.0 / image_params.width, 240.0 / image_params.height};

        public:
            std::string input;
            std::string plyfile;
            bool grey = true, rgb = true, depth = true, gt = true;
            bool GetFrame(const std::string &dirname,
                          int frame_no,
                          SLAMFile &file,
                          std::ifstream& infile,
                          CameraSensor* rgb_sensor = nullptr,
                          CameraSensor* grey_sensor = nullptr,
                          DepthSensor* depth_sensor = nullptr,
                          GroundTruthSensor* gt_sensor = nullptr);
            explicit VolumeDeformReader(const std::string& name) : DatasetReader(name) {

                this->addParameter(TypedParameter<std::string>("i", "input-directory",
                                                               "path of the VolumeDeform dataset directory",
                                                               &this->input, nullptr));

                this->addParameter(TypedParameter<bool>("grey", "grey",
                                                        "set to true or false to specify if the GREY "
                                                        "stream need to be include in the slam file.",
                                                        &this->grey, nullptr));

                this->addParameter(TypedParameter<bool>("rgb", "rgb",
                                                        "set to true or false to specify if the RGB "
                                                        "stream need to be include in the slam file.",
                                                        &this->rgb, nullptr));

                this->addParameter(TypedParameter<bool>("depth", "depth",
                                                        "set to true or false to specify if the DEPTH "
                                                        "stream need to be include in the slam file.",
                                                        &this->depth, nullptr));

                this->addParameter(TypedParameter<bool>("gt", "gt",
                                                        "set to true or false to specify if the GROUNDTRUTH "
                                                        "POSE stream need to be include in the slam file.",
                                                        &this->gt, nullptr));

                this->addParameter(TypedParameter<std::string>("ply", "ply-file",
                                                               "When a PLY file is specified, the GROUNDTRUTH POINT "
                                                               "CLOUD will be included in the slam file.",
                                                               &this->plyfile, nullptr));
            }

            SLAMFile* GenerateSLAMFile() override;
        };
    }  // namespace io
}  // namespace slambench

#endif /* FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_VOLUMEDEFORM_H_ */
