/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_SVO_H_
#define FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_SVO_H_

#include <io/sensor/CameraSensor.h>
#include <io/sensor/DepthSensor.h>
#include <io/sensor/GroundTruthSensor.h>
#include <io/sensor/Sensor.h>

#include <string>

#include "DatasetReader.h"

#include <ParameterComponent.h>
#include <ParameterManager.h>
#include <Parameters.h>


namespace slambench {

  namespace io {

    class SVOReader : public DatasetReader {
     private:
      static constexpr CameraSensor::intrinsics_t svo_grey = {0.419547872, 0.657291667, 0.5, 0.5}; // ATAN
      //  static constexpr CameraSensor::intrinsics_t svo_grey   = { 315.5, 315.5, 376.0, 240.0 }; // Pinhole

      static constexpr float translation[] = {0.1131, 0.1131, 2.0};  // x, y, z
      static constexpr float rotation[] = {0.0, 0.9675388, 0.2527226, 0.0};  // w, x, y, z

     public:
      std::string input;

      explicit SVOReader(const std::string& name) : DatasetReader(name) {
        this->addParameter(TypedParameter<std::string>(
            "i", "input-directory",
            "path of the SVO dataset directory",
            &this->input, nullptr));
      }

      SLAMFile* GenerateSLAMFile() override;
    };

  }  // namespace io
}  // namespace slambench

#endif /* FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_SVO_H_ */
