/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


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
	bool SerialiseSensorSpecific(Serialiser* serialiser, const Sensor* sensor) override {
		(void)serialiser;
		(void)sensor;
		return false;
	}

};


class EventCameraSensorDeserialiser : public SensorDeserialiser {
	bool InstantiateSensor(const Sensor::sensor_name_t &sensor_name, const Sensor::sensor_type_t &type, Sensor** s) override {
		(void)type;
		(void)s;
		(void)sensor_name;
		return false;
	}
	bool DeserialiseSensorSpecific(Deserialiser* d, Sensor* s) override {
		(void)d;
		(void)s;
		return false;
	}


};

static slambench::io::SensorDatabaseRegistration evcam_reg(EventCameraSensor::kEventCameraType, slambench::io::SensorDatabaseEntry(new EventCameraSensorSerialiser(), new EventCameraSensorDeserialiser(), false, true));
