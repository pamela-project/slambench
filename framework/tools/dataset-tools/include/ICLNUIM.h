/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#ifndef FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_ICLNUIM_H_
#define FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_ICLNUIM_H_

#include <ParameterManager.h>
#include <ParameterComponent.h>
#include <Parameters.h>

#include <io/sensor/Sensor.h>
#include <io/sensor/CameraSensor.h>
#include <io/sensor/DepthSensor.h>
#include <io/sensor/GroundTruthSensor.h>
#include "DatasetReader.h"

struct float3 { float x, y, z; };

namespace slambench {

namespace io {

class ICLNUIMReader :  public DatasetReader {

public :
	std::string input;
	bool grey = true, rgb = true, depth = true, gt = true;
	std::string plyfile = "";
	bool positive_focal = false;

	ICLNUIMReader (std::string name) : DatasetReader(name) {
		this->addParameter(TypedParameter<std::string>("i",     "input-directory",       "path of the ICLNUIM dataset directory",   &this->input, NULL));
		this->addParameter(TypedParameter<bool>("grey",     "grey",       "set to true or false to specify if the GREY stream need to be include in the slam file.",   &this->grey, NULL));
		this->addParameter(TypedParameter<bool>("rgb",     "rgb",       "set to true or false to specify if the RGB stream need to be include in the slam file.",   &this->rgb, NULL));
		this->addParameter(TypedParameter<bool>("depth",     "depth",       "set to true or false to specify if the DEPTH stream need to be include in the slam file.",   &this->depth, NULL));
		this->addParameter(TypedParameter<bool>("gt",     "gt",       "set to true or false to specify if the GROUNDTRUTH POSE stream need to be include in the slam file.",   &this->gt, NULL));
		this->addParameter(TypedParameter<std::string>("ply",     "ply-file",       "When a PLY file is specified, the GROUNDTRUTH POINT CLOUD will be included in the slam file.",   &this->plyfile, NULL));
		this->addParameter(TypedParameter<bool>("pf",     "positive-focal",       "This is a workaround to correct the ICLNUIM to a positive focal lenght.",   &this->positive_focal, NULL));

	}

	SLAMFile* GenerateSLAMFile () ;

private :
	CameraSensor *rgb_sensor = nullptr;
	DepthSensor *depth_sensor = nullptr;
	CameraSensor *grey_sensor =  nullptr;
	GroundTruthSensor *gt_sensor =  nullptr;
	void AddSensors(SLAMFile &file);
	bool GetFrame(const std::string &dirname, SLAMFile &file, int frame_no) ;
	bool AddFrames(const std::string &dirname, SLAMFile &file) ;

};

}
}

#endif /* FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_ICLNUIM_H_ */
