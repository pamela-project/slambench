/*

 Copyright (c) 2019 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_UZHFPV_H_
#define FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_UZHFPV_H_

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

    class UZHFPVReader : public DatasetReader {
     private:
//      static constexpr CameraSensor::intrinsics_t fr1_intrinsics_rgb = {0.9375, 1.25, 0.5, 0.5};
//      static constexpr DepthSensor::intrinsics_t fr1_intrinsics_depth = {0.9375, 1.25, 0.5, 0.5};

     public:
      std::string input;
      bool imu = true, gt = false, stereo = false, events = false;

      explicit UZHFPVReader(const std::string& name) : DatasetReader(name) {

        this->addParameter(TypedParameter<std::string>("i", "input-directory",
                                                       "path of the UZHFPV dataset directory",
                                                       &this->input,
                                                       nullptr));

        this->addParameter(TypedParameter<bool>("s", "stereo",
                                                "set to true or false to specify if the STEREO "
                                                "stream need to be include in the slam file.",
                                                &this->stereo,
                                                nullptr));

        this->addParameter(TypedParameter<bool>("e", "event",
                                                "set to true or false to specify if the EVENT "
                                                "stream need to be include in the slam file.",
                                                &this->events,
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

#endif /* FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_UZHFPV_H_ */
