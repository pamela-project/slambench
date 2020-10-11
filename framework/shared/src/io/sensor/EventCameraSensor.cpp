/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include "io/serialisation/Serialiser.h"
#include "io/deserialisation/Deserialiser.h"
#include "io/sensor/EventCameraSensor.h"
#include "io/sensor/SensorDatabase.h"


using namespace slambench::io;

const Sensor::sensor_type_t EventCameraSensor::kEventCameraType = "EventCamera";

EventCameraSensor::EventCameraSensor(const sensor_name_t &sensor_name) : Sensor(sensor_name, kEventCameraType) {

}

size_t EventCameraSensor::GetCoordinateSize() const {
	if(Width <= UINT8_MAX && Height <= UINT8_MAX) return 1;
	if(Width <= UINT16_MAX && Height <= UINT16_MAX) return 2;
	return 4;
}

size_t EventCameraSensor::GetFrameSize(const SLAMFrame *frame) const {
	// X, Y, on/off
	(void)frame;
	return (GetCoordinateSize() * 2) + 1;
}

class EventCameraSensorSerialiser : public SensorSerialiser {
  bool SerialiseSensorSpecific(Serialiser* serialiser, const Sensor* s) override {
		EventCameraSensor *sensor = (EventCameraSensor*)s;
		serialiser->Write(&sensor->Width, sizeof(sensor->Width));
		serialiser->Write(&sensor->Height, sizeof(sensor->Height));
		return true;
	}

};


class EventCameraSensorDeserialiser : public SensorDeserialiser {
	bool InstantiateSensor(const Sensor::sensor_name_t &sensor_name, const Sensor::sensor_type_t &type, Sensor** s) override {
		if(type != EventCameraSensor::kEventCameraType) {
			return false;
		}

		*s = new EventCameraSensor(sensor_name);

		return true;
	}

	bool DeserialiseSensorSpecific(const Deserialiser* deserialiser, Sensor* s) override {
		EventCameraSensor *sensor = (EventCameraSensor*)s;

		assert(sensor->GetType() == EventCameraSensor::kEventCameraType);

		deserialiser->Read(&sensor->Width, sizeof(sensor->Width));
		deserialiser->Read(&sensor->Height, sizeof(sensor->Height));
          	
		return true;
	}


};

static slambench::io::SensorDatabaseRegistration evcam_reg(EventCameraSensor::kEventCameraType, slambench::io::SensorDatabaseEntry(new EventCameraSensorSerialiser(), new EventCameraSensorDeserialiser(), false, true));
