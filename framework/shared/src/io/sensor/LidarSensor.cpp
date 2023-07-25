/*

 Copyright (c) 2023 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include "io/SLAMFrame.h"
#include "io/sensor/LidarSensor.h"
#include "io/sensor/SensorDatabase.h"
#include "io/serialisation/Serialiser.h"
#include "io/deserialisation/Deserialiser.h"

#include <cassert>

using namespace slambench::io;

const Sensor::sensor_type_t LidarSensor::kLidarType = "Lidar";

LidarSensor::LidarSensor(const sensor_name_t &sensor_name) : Sensor(sensor_name, kLidarType)
{}

size_t LidarSensor::GetFrameSize(const SLAMFrame *frame) const {
	return frame->GetVariableSize();
}

void LidarSensor::CopyHorizontalFoV(const fov_t &other) {
	for(unsigned int i = 0; i < 2 ; ++i) {
		HorizontalFoV[i] = other[i];
	}
}

void LidarSensor::CopyVerticalFoV(const fov_t &other) {
	for(unsigned int i = 0; i < 2 ; ++i) {
		VerticalFoV[i] = other[i];
	}
}

class LidarSerialiser : public SensorSerialiser {
	bool SerialiseSensorSpecific(Serialiser* serialiser, const Sensor* s) override {
		LidarSensor *sensor = (LidarSensor*)s;
		
		serialiser->Write(&sensor->PointDensity, sizeof(sensor->PointDensity));
		serialiser->Write(&sensor->SideLap, sizeof(sensor->SideLap));
		serialiser->Write(&sensor->AltitudeAboveGround, sizeof(sensor->AltitudeAboveGround));
		serialiser->Write(&sensor->HorizontalFoV, sizeof(sensor->HorizontalFoV));
		serialiser->Write(&sensor->VerticalFoV, sizeof(sensor->VerticalFoV));
		serialiser->Write(&sensor->HorizontalAngResolution, sizeof(sensor->HorizontalAngResolution));
		serialiser->Write(&sensor->VerticalAngResolution, sizeof(sensor->VerticalAngResolution));
		serialiser->Write(&sensor->BeamNum, sizeof(sensor->BeamNum));

		return true;
	}
};

class LidarDeserialiser : public SensorDeserialiser {
	bool InstantiateSensor(const Sensor::sensor_name_t &sensor_name, const Sensor::sensor_type_t &type, Sensor** s) override {
		if(type == LidarSensor::kLidarType) {
			*s = new LidarSensor(sensor_name);
			return true;
		} else {
			return false;
		}
	}

	bool DeserialiseSensorSpecific(const Deserialiser* deserialiser, Sensor* s) override {
		LidarSensor *sensor = (LidarSensor*)s;
		
		assert(sensor->GetType() == LidarSensor::kLidarType);
		
		deserialiser->Read(&sensor->PointDensity, sizeof(sensor->PointDensity));
		deserialiser->Read(&sensor->SideLap, sizeof(sensor->SideLap));
		deserialiser->Read(&sensor->AltitudeAboveGround, sizeof(sensor->AltitudeAboveGround));
		deserialiser->Read(&sensor->HorizontalFoV, sizeof(sensor->HorizontalFoV));
		deserialiser->Read(&sensor->VerticalFoV, sizeof(sensor->VerticalFoV));
		deserialiser->Read(&sensor->HorizontalAngResolution, sizeof(sensor->HorizontalAngResolution));
		deserialiser->Read(&sensor->VerticalAngResolution, sizeof(sensor->VerticalAngResolution));
		deserialiser->Read(&sensor->BeamNum, sizeof(sensor->BeamNum));

		return true;
	}
};

static slambench::io::SensorDatabaseRegistration lidar_reg(LidarSensor::kLidarType, slambench::io::SensorDatabaseEntry(new LidarSerialiser(), new LidarDeserialiser(), false, false));
