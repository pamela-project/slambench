/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include "io/sensor/CameraSensorFinder.h"
#include "io/sensor/CameraSensor.h"
#include "io/sensor/DepthSensor.h"
#include "io/sensor/SensorCollection.h"

using namespace slambench::io;

CameraSensorFinder::CameraSensorFinder()
{
	PrepareEvaluators();
}

std::vector<CameraSensor*> CameraSensorFinder::Find(const SensorCollection& sensors, const std::initializer_list<CameraSensorFilter> &filters)
{
	std::vector<CameraSensor*> output;
	
	for(auto sensor : sensors) {
		// this is still a nightmare but at least it only happens here
		if(sensor->GetType() == CameraSensor::kCameraType || sensor->GetType() == DepthSensor::kDepthType) {
			
			CameraSensor *cam = static_cast<CameraSensor*>(sensor);
			
			bool success = true;
			for(auto filter : filters) {
				success &= TestFilter(cam, filter);
			}
			if(success) {
				output.push_back(cam);
			}
			
		}
	}
	
	return output;
}

CameraSensor* CameraSensorFinder::FindOne(const SensorCollection& sensors, const std::initializer_list<CameraSensorFilter>& filters)
{
	auto found_sensors = Find(sensors, filters);
	if(found_sensors.empty()) {
		return nullptr;
	} else {
		return found_sensors.front();
	}
}

bool CameraSensorFinder::TestFilter(const CameraSensor* sensor, const CameraSensorFilter& filter)
{
	return filter_evaluators_.at(filter.first)(sensor, filter.second);
}

void CameraSensorFinder::PrepareEvaluators()
{
	filter_evaluators_["width"] = [](const CameraSensor *sensor, const std::string &value) -> bool {return sensor->Width == std::stoul(value);};
	filter_evaluators_["height"] = [](const CameraSensor *sensor, const std::string &value) -> bool {return sensor->Height == std::stoul(value);};
	filter_evaluators_["name"] = [](const CameraSensor *sensor, const std::string &value) -> bool {return sensor->getName() == value;};
	filter_evaluators_["pixel_format"] = [](const CameraSensor *sensor, const std::string &value) -> bool {return sensor->PixelFormat == pixelformat::Parse(value);};
	filter_evaluators_["camera_type"] = [](const CameraSensor *sensor, const std::string &value) -> bool {return pixelformat::TypeOf(sensor->PixelFormat) == value;};
	filter_evaluators_["frame_format"] = [](const CameraSensor *sensor, const std::string &value) -> bool {return sensor->FrameFormat == frameformat::Parse(value);};
}
