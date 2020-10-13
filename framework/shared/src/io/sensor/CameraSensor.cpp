/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include "io/serialisation/Serialiser.h"
#include "io/deserialisation/Deserialiser.h"
#include "io/sensor/CameraSensor.h"
#include "io/sensor/SensorDatabase.h"

#include <cassert>
#include <Parameters.h>

using namespace slambench::io;

const Sensor::sensor_type_t CameraSensor::kCameraType = "Camera";

CameraSensor::CameraSensor(const Sensor::sensor_name_t  &name,  const Sensor::sensor_type_t &sensor_type) :
		Sensor(name, sensor_type) ,
		Width (0),
		Height (0),
		FrameFormat (slambench::io::frameformat::UNKNOWN),
		PixelFormat(slambench::io::pixelformat::UNKNOWN),
		DistortionType(NoDistortion)
{
	this->addParameter(TypedParameter<intrinsics_t>("ip", "intrinsics-parameters","Focal length and Principal Point Offset : fx,fy,cx,cy", &(this->Intrinsics),nullptr));
}

size_t CameraSensor::GetFrameSize(const SLAMFrame *frame) const {
	(void)frame;
	return Width * Height * pixelformat::GetPixelSize(PixelFormat);
}

void CameraSensor::CopyRadialTangentialDistortion(const distortion_coefficients_t &other) {
	for(unsigned int i = 0; i < 5 ; ++i) {
		RadialTangentialDistortion[i] = other[i];
	}
}

void CameraSensor::CopyEquidistantDistortion(const distortion_coefficients_t &other) {
  for(unsigned int i = 0; i < 4 ; ++i) {
    EquidistantDistortion[i] = other[i];
  }
}
void CameraSensor::CopyDistortion(const distortion_coefficients_t &other, const distortion_type_t& type) {
  for(unsigned int i = 0; i < 5 ; ++i) {
    Distortion[i] = other[i];
  }
}

void CameraSensor::CopyIntrinsics(const intrinsics_t &other) {
	for(unsigned int i = 0; i < 4 ; ++i) {
		Intrinsics[i] = other[i];
	}
}

void CameraSensor::CopyIntrinsics(const CameraSensor* other) {
	CopyIntrinsics(other->Intrinsics);
}

class CameraSensorSerialiser : public SensorSerialiser {
	bool SerialiseSensorSpecific(Serialiser* serialiser, const Sensor* s) override {
		CameraSensor *sensor = (CameraSensor*)s;
		
		serialiser->Write(&sensor->FrameFormat, sizeof(sensor->FrameFormat));
		serialiser->Write(&sensor->PixelFormat, sizeof(sensor->PixelFormat));
		serialiser->Write(&sensor->Width, sizeof(sensor->Width));
		serialiser->Write(&sensor->Height, sizeof(sensor->Height));
		serialiser->Write(&sensor->Intrinsics, sizeof(sensor->Intrinsics));
		serialiser->Write(&sensor->DistortionType, sizeof(sensor->DistortionType));
		serialiser->Write(&sensor->RadialTangentialDistortion, sizeof(sensor->RadialTangentialDistortion));
        serialiser->Write(&sensor->EquidistantDistortion, sizeof(sensor->EquidistantDistortion));
        serialiser->Write(&sensor->Distortion, sizeof(sensor->Distortion));
          return true;
	}
};

class CameraSensorDeserialiser : public SensorDeserialiser {
	
	bool InstantiateSensor(const Sensor::sensor_name_t &sensor_name, const Sensor::sensor_type_t &type, Sensor** s) override {
		if(type == CameraSensor::kCameraType) {
			*s = new CameraSensor(sensor_name, type);
			return true;
		} else {
			return false;
		}
	}

	bool DeserialiseSensorSpecific(const Deserialiser* deserialiser, Sensor* s) override {
		CameraSensor *sensor = (CameraSensor*)s;
		
		assert(sensor->GetType() == CameraSensor::kCameraType);
		
		deserialiser->Read(&sensor->FrameFormat, sizeof(sensor->FrameFormat));
		deserialiser->Read(&sensor->PixelFormat, sizeof(sensor->PixelFormat));
		deserialiser->Read(&sensor->Width, sizeof(sensor->Width));
		deserialiser->Read(&sensor->Height, sizeof(sensor->Height));
		deserialiser->Read(&sensor->Intrinsics, sizeof(sensor->Intrinsics));
		deserialiser->Read(&sensor->DistortionType, sizeof(sensor->DistortionType));
		deserialiser->Read(&sensor->RadialTangentialDistortion, sizeof(sensor->RadialTangentialDistortion));
        deserialiser->Read(&sensor->EquidistantDistortion, sizeof(sensor->EquidistantDistortion));
        deserialiser->Read(&sensor->Distortion, sizeof(sensor->Distortion));

		return true;
	}
};

static slambench::io::SensorDatabaseRegistration camera_reg(CameraSensor::kCameraType, slambench::io::SensorDatabaseEntry(new CameraSensorSerialiser(), new CameraSensorDeserialiser(), false, false));
