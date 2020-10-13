/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include "io/sensor/GroundTruthSensor.h"
#include "io/sensor/SensorDatabase.h"

using namespace slambench::io;

const Sensor::sensor_type_t GroundTruthSensor::kGroundTruthTrajectoryType = "GroundTruthTrajectory";

GroundTruthSensor::GroundTruthSensor(const Sensor::sensor_name_t &sensor_name) : Sensor(sensor_name, kGroundTruthTrajectoryType) {
}

size_t GroundTruthSensor::GetFrameSize(const SLAMFrame *frame) const {
	(void)frame;
	return sizeof(pose_t);
}

class GTSerialiser : public SensorSerialiser {
	bool SerialiseSensorSpecific(Serialiser* serialiser, const Sensor* sensor) override {
		// nothing to do
		(void)serialiser;
		(void)sensor;
		return true;
	}
};

class GTDeserialiser : public SensorDeserialiser {
	bool InstantiateSensor(const Sensor::sensor_name_t &sensor_name, const Sensor::sensor_type_t &type, Sensor** s) override {
		if(type != GroundTruthSensor::kGroundTruthTrajectoryType) {
			return false;
		}
		
		*s = new GroundTruthSensor(sensor_name);
		
		return true;
	}

	bool DeserialiseSensorSpecific(const Deserialiser* d, Sensor* s) override {
		// nothing to do
		(void)d;
		(void)s;
		return true;
	}
};

static slambench::io::SensorDatabaseRegistration gt_reg(GroundTruthSensor::kGroundTruthTrajectoryType, slambench::io::SensorDatabaseEntry(new GTSerialiser(), new GTDeserialiser(), true, false));
