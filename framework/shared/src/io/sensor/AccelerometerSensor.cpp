/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include "io/sensor/AccelerometerSensor.h"
#include "io/sensor/SensorDatabase.h"

using namespace slambench::io;

const Sensor::sensor_type_t AccelerometerSensor::kAccType = "Accelerometer";

AccelerometerSensor::AccelerometerSensor(const sensor_name_t &sensor_name) : Sensor(sensor_name, kAccType) {

}
size_t AccelerometerSensor::GetFrameSize(const SLAMFrame *frame) const  {
	(void)frame;
	return 3 * sizeof(float);
}

class ACCSerialiser : public SensorSerialiser {
	bool SerialiseSensorSpecific(Serialiser* serialiser, const Sensor* sensor) override {
		// nothing to do
		(void)serialiser;
		(void)sensor;
		return true;
	}
};

class ACCDeserialiser : public SensorDeserialiser {
	bool InstantiateSensor(const Sensor::sensor_name_t &sensor_name,const Sensor::sensor_type_t &type, Sensor** s) override {
		if(type != AccelerometerSensor::kAccType) {
			return false;
		}

		*s = new AccelerometerSensor(sensor_name);

		return true;
	}

	bool DeserialiseSensorSpecific(Deserialiser* d, Sensor* s) override {
		// nothing to do
		(void)d;
		(void)s;
		return true;
	}
};

static slambench::io::SensorDatabaseRegistration acc_reg(AccelerometerSensor::kAccType, slambench::io::SensorDatabaseEntry(new ACCSerialiser(), new ACCDeserialiser(), false, false));


