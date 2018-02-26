/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#ifndef FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_SVO_H_
#define FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_SVO_H_



#include <ParameterManager.h>
#include <ParameterComponent.h>
#include <Parameters.h>

#include <io/sensor/Sensor.h>
#include <io/sensor/CameraSensor.h>
#include <io/sensor/DepthSensor.h>
#include <io/sensor/GroundTruthSensor.h>
#include "../../dataset-tools/include/DatasetReader.h"

namespace slambench {

namespace io {

class SVOReader :  public DatasetReader {

private :



public :
	std::string input;

	SVOReader (std::string name) : DatasetReader(name) {
		this->addParameter(TypedParameter<std::string>("i",     "input-directory",       "path of the SVO dataset directory",   &this->input, NULL));

	}

	SLAMFile* GenerateSLAMFile () ;


};

}
}



#endif /* FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_SVO_H_ */
