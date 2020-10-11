/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include "io/SLAMFrame.h"
#include "io/sensor/PointCloudSensor.h"
#include "io/sensor/SensorDatabase.h"

using namespace slambench::io;

const Sensor::sensor_type_t PointCloudSensor::kPointCloudType = "PointCloud";

PointCloudSensor::PointCloudSensor(const sensor_name_t &sensor_name) : Sensor(sensor_name, kPointCloudType)
{}

size_t PointCloudSensor::GetFrameSize(const SLAMFrame *frame) const {
	return frame->GetVariableSize();
}

class PCSerialiser : public SensorSerialiser {
	bool SerialiseSensorSpecific(Serialiser* serialiser, const Sensor* sensor) override {
		// nothing to do
		(void)serialiser;
		(void)sensor;
		return true;
	}
};

class PCDeserialiser : public SensorDeserialiser {
	bool InstantiateSensor(const Sensor::sensor_name_t &sensor_name, const Sensor::sensor_type_t &type, Sensor** s) override {
		if(type != PointCloudSensor::kPointCloudType) {
			return false;
		}

		*s = new PointCloudSensor(sensor_name);

		return true;
	}

	bool DeserialiseSensorSpecific(const Deserialiser* d, Sensor* s) override {
		// nothing to do
		(void)d;
		(void)s;
		return true;
	}
};

static slambench::io::SensorDatabaseRegistration rgb_reg(PointCloudSensor::kPointCloudType, slambench::io::SensorDatabaseEntry(new PCSerialiser(), new PCDeserialiser(), true, true));
