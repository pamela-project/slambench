/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_EUROCMAV_H_
#define FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_EUROCMAV_H_

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

    class EUROCMAVReader : public DatasetReader {
     private:
      static constexpr CameraSensor::intrinsics_t fr1_intrinsics_rgb = {0.9375, 1.25, 0.5, 0.5};
      static constexpr DepthSensor::intrinsics_t fr1_intrinsics_depth = {0.9375, 1.25, 0.5, 0.5};

     public:
      std::string input;
      bool imu = true, gt = true, stereo = true, rgb = false;

      explicit EUROCMAVReader(const std::string& name) : DatasetReader(name) {

        this->addParameter(TypedParameter<std::string>("i", "input-directory",
                                                       "path of the EUROCMAV dataset directory",
                                                       &this->input,
                                                       nullptr));

        this->addParameter(TypedParameter<bool>("sgrey", "stereo-grey",
                                                "set to true or false to specify if the STEREO GREY "
                                                "stream need to be include in the slam file.",
                                                &this->stereo,
                                                nullptr));

        this->addParameter(TypedParameter<bool>("srgb", "srgb",
                                                "set to true or false to specify if the STEREO RGB "
                                                "stream need to be include in the slam file.",
                                                &this->rgb,
                                                nullptr));

        this->addParameter(TypedParameter<bool>("gt", "gt",
                                                "set to true or false to specify if the GROUNDTRUTH POSE "
                                                "stream need to be include in the slam file.",
                                                &this->gt,
                                                nullptr));

        this->addParameter(TypedParameter<bool>("imu", "imu",
                                                "set to true or false to specify if the IMU "
                                                "stream need to be include in the slam file.",
                                                &this->imu,
                                                nullptr));
      }

      SLAMFile* GenerateSLAMFile() override;
    };

  }  // namespace io
}  // namespace slambench

#endif /* FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_EUROCMAV_H_ */
