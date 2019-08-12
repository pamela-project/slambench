/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_ICL_H_
#define FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_ICL_H_

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

    class ICLReader : public DatasetReader {
     private:
      static constexpr CameraSensor::intrinsics_t fr1_intrinsics_rgb = {0.9375, 1.25, 0.5, 0.5};
      static constexpr DepthSensor::intrinsics_t fr1_intrinsics_depth = {0.9375, 1.25, 0.5, 0.5};

     public:
      std::string input;
      bool grey = true, rgb = true, depth = true, gt = true, accelerometer = false;

      explicit ICLReader(const std::string& name) : DatasetReader(name) {

        this->addParameter(TypedParameter<std::string>("i", "input-directory",
                                                       "path of the ICL dataset directory",
                                                       &this->input,
                                                       nullptr));

        this->addParameter(TypedParameter<bool>("grey", "grey",
                                                "set to true or false to specify if the GREY "
                                                "stream need to be include in the slam file.",
                                                &this->grey,
                                                nullptr));

        this->addParameter(TypedParameter<bool>("rgb", "rgb",
                                                "set to true or false to specify if the RGB "
                                                "stream need to be include in the slam file.",
                                                &this->rgb,
                                                nullptr));

        this->addParameter(TypedParameter<bool>("depth", "depth",
                                                "set to true or false to specify if the DEPTH "
                                                "stream need to be include in the slam file.",
                                                &this->depth,
                                                nullptr));

        this->addParameter(TypedParameter<bool>("gt", "gt",
                                                "set to true or false to specify if the GROUNDTRUTH POSE "
                                                "stream need to be include in the slam file.",
                                                &this->gt,
                                                nullptr));

        this->addParameter(TypedParameter<bool>("acc", "accelerometer",
                                                "set to true or false to specify if the ACCELEROMETER "
                                                "stream need to be include in the slam file.",
                                                &this->accelerometer,
                                                nullptr));
      }

      SLAMFile* GenerateSLAMFile() override;
    };

  }  // namespace io
}  // namespace slambench

#endif /* FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_ICL_H_ */
