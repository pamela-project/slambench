/*

 Copyright (c) 2019 Intel Corp.

 This code is licensed under the MIT License.

 */


#include "io/sensor/OdomSensor.h"
#include "io/sensor/SensorDatabase.h"

using namespace slambench::io;

const Sensor::sensor_type_t OdomSensor::kOdomType = "Odom";

OdomSensor::OdomSensor(const sensor_name_t &sensor_name) : Sensor(sensor_name, kOdomType) {

}
size_t OdomSensor::GetFrameSize(const SLAMFrame *frame) const  {
	(void)frame;
	return 13 * sizeof(float);
}

class OdomSerialiser : public SensorSerialiser {
	bool SerialiseSensorSpecific(Serialiser* serialiser, const Sensor* sensor) override {
		// nothing to do
		(void)serialiser;
		(void)sensor;
		return true;
	}
};

class OdomDeserialiser : public SensorDeserialiser {
	bool InstantiateSensor(const Sensor::sensor_name_t &sensor_name,const Sensor::sensor_type_t &type, Sensor** s) override {
		if(type != OdomSensor::kOdomType) {
			return false;
		}

		*s = new OdomSensor(sensor_name);

		return true;
	}

	bool DeserialiseSensorSpecific(const Deserialiser* d, Sensor* s) override {
		// nothing to do
		(void)d;
		(void)s;
		return true;
	}
};

static slambench::io::SensorDatabaseRegistration odom_reg(OdomSensor::kOdomType, slambench::io::SensorDatabaseEntry(new OdomSerialiser(), new OdomDeserialiser(), false, false));


