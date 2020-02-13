/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_ETHI_H_
#define FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_ETHI_H_

#include <ParameterComponent.h>
#include <ParameterManager.h>
#include <Parameters.h>

#include <io/sensor/CameraSensor.h>
#include <io/sensor/DepthSensor.h>
#include <io/sensor/GroundTruthSensor.h>
#include "DatasetReader.h"

namespace slambench {

  namespace io {

    class ETHIReader : public DatasetReader {

     public:

      std::string dataset = "";
        explicit ETHIReader(const std::string &name) : DatasetReader(name) {
        this->addParameter(TypedParameter<std::string>("dataset", "base-dataset", "If this is iclnuim or tum",
                                                &this->dataset, nullptr));
      }

      SLAMFile *GenerateSLAMFile() override;
    };

  }  // namespace io
}  // namespace slambench

#endif /* FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_ETHI_H_ */
