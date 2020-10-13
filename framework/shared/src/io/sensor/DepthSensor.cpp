/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include <Parameters.h>
#include "io/serialisation/Serialiser.h"
#include "io/deserialisation/Deserialiser.h"
#include "io/sensor/DepthSensor.h"
#include "io/sensor/SensorDatabase.h"

#include <cassert>

using namespace slambench::io;

const Sensor::sensor_type_t DepthSensor::kDepthType = "Depth";

DepthSensor::DepthSensor(const Sensor::sensor_name_t &sensor_name) : CameraSensor(sensor_name, kDepthType) {
    this->addParameter(TypedParameter<disparity_params_t>("dip", "disparity-params","Don't know what to say", &(this->DisparityParams),NULL));
}

void DepthSensor::CopyDisparityParams(const disparity_params_t &other) {
	for(int i = 0; i < 2; ++i) {
		DisparityParams[i] = other[i];
	}
}


void DepthSensor::CopyDisparityParams(const DepthSensor* other) {
	CopyDisparityParams(other->DisparityParams);
}

class DepthSensorSerialiser : public SensorSerialiser {
	bool SerialiseSensorSpecific(Serialiser* serialiser, const Sensor* s) override {
		DepthSensor *sensor = (DepthSensor*)s;
		
		assert(sensor->GetType() == DepthSensor::kDepthType);
		
		serialiser->Write(&sensor->FrameFormat, sizeof(sensor->FrameFormat));
		serialiser->Write(&sensor->PixelFormat, sizeof(sensor->PixelFormat));
		serialiser->Write(&sensor->Width, sizeof(sensor->Width));
		serialiser->Write(&sensor->Height, sizeof(sensor->Height));
		serialiser->Write(&sensor->Intrinsics, sizeof(sensor->Intrinsics));
		serialiser->Write(&sensor->DisparityParams, sizeof(sensor->DisparityParams));
		serialiser->Write(&sensor->DisparityType, sizeof(sensor->DisparityType));
        serialiser->Write(&sensor->DistortionType, sizeof(sensor->DistortionType));
        serialiser->Write(&sensor->RadialTangentialDistortion, sizeof(sensor->RadialTangentialDistortion));

		return true;
	}
};

class DepthSensorDeserialiser : public SensorDeserialiser {
	
	bool InstantiateSensor(const Sensor::sensor_name_t &sensor_name, const Sensor::sensor_type_t & type, Sensor** s) override {
		if(type == DepthSensor::kDepthType) {
			*s = new DepthSensor(sensor_name);
			return true;
		} else {
			return false;
		}
	}

	bool DeserialiseSensorSpecific(const Deserialiser* deserialiser, Sensor* s) override {
		DepthSensor *sensor = (DepthSensor*)s;
		
		assert(sensor->GetType() == DepthSensor::kDepthType);
		
		deserialiser->Read(&sensor->FrameFormat, sizeof(sensor->FrameFormat));
		deserialiser->Read(&sensor->PixelFormat, sizeof(sensor->PixelFormat));
		deserialiser->Read(&sensor->Width, sizeof(sensor->Width));
		deserialiser->Read(&sensor->Height, sizeof(sensor->Height));
		deserialiser->Read(&sensor->Intrinsics, sizeof(sensor->Intrinsics));
		deserialiser->Read(&sensor->DisparityParams, sizeof(sensor->DisparityParams));
		deserialiser->Read(&sensor->DisparityType, sizeof(sensor->DisparityType));
        deserialiser->Read(&sensor->DistortionType, sizeof(sensor->DistortionType));
        deserialiser->Read(&sensor->RadialTangentialDistortion, sizeof(sensor->RadialTangentialDistortion));
		
		return true;
	}
};

static slambench::io::SensorDatabaseRegistration depth_reg (DepthSensor::kDepthType, slambench::io::SensorDatabaseEntry(new DepthSensorSerialiser(), new DepthSensorDeserialiser()));
