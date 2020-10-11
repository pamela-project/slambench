/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_ICLNUIM_H_
#define FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_ICLNUIM_H_

#include <ParameterComponent.h>
#include <ParameterManager.h>
#include <Parameters.h>

#include <io/sensor/CameraSensor.h>
#include <io/sensor/DepthSensor.h>
#include <io/sensor/GroundTruthSensor.h>
#include <fstream>
#include "DatasetReader.h"

namespace slambench {

  namespace io {

    class ICLNUIMReader : public DatasetReader {
     private:
      CameraSensor *rgb_sensor = nullptr;
      DepthSensor *depth_sensor = nullptr;
      CameraSensor *grey_sensor = nullptr;
      GroundTruthSensor *gt_sensor = nullptr;

      std::ifstream istream;
      static constexpr DepthSensor::disparity_params_t disparity_params = {0.0002, 0.0};
      static constexpr DepthSensor::disparity_type_t disparity_type = DepthSensor::affine_disparity;

      void AddSensors(SLAMFile &file);
      bool GetFrame(const std::string &dirname, SLAMFile &file, int frame_no);
      bool AddFrames(const std::string &dirname, SLAMFile &file);

     public:
      static constexpr image_params_t image_params = { 640, 480, 1, 5000.0 };

        std::string input;
      bool grey = true, rgb = true, depth = true, gt = true;
      std::string plyfile = "";
      bool positive_focal = false;

      explicit ICLNUIMReader(const std::string &name) : DatasetReader(name) {
        this->addParameter(TypedParameter<std::string>("i", "input-directory",
                                                       "path of the ICLNUIM dataset directory",
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
                                                "set to true or false to specify if the "
                                                "GROUNDTRUTH POSE stream need to be include in the slam file.",
                                                &this->gt, nullptr));

        this->addParameter(TypedParameter<std::string>("ply", "ply-file",
                                                       "When a PLY file is specified, the GROUNDTRUTH POINT CLOUD "
                                                       "will be included in the slam file.",
                                                       &this->plyfile, nullptr));

        this->addParameter(TypedParameter<bool>("pf", "positive-focal",
                                                "This is a workaround to correct the ICLNUIM "
                                                "to a positive focal length.",
                                                &this->positive_focal, nullptr));
      }

      SLAMFile *GenerateSLAMFile() override;
    };

  }  // namespace io
}  // namespace slambench

#endif /* FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_ICLNUIM_H_ */
