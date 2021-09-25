/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include "io/sensor/AccelerometerSensor.h"
#include "io/sensor/SensorDatabase.h"
#include "io/serialisation/Serialiser.h"
#include "io/deserialisation/Deserialiser.h"

using namespace slambench::io;

const Sensor::sensor_type_t AccelerometerSensor::kAccType = "Accelerometer";

AccelerometerSensor::AccelerometerSensor(const sensor_name_t &sensor_name) : Sensor(sensor_name, kAccType) {

}
size_t AccelerometerSensor::GetFrameSize(const SLAMFrame *frame) const  {
	(void)frame;
	return 3 * sizeof(float);
}

class ACCSerialiser : public SensorSerialiser {
	bool SerialiseSensorSpecific(Serialiser* serialiser, const Sensor* s) override {
		AccelerometerSensor *sensor = (AccelerometerSensor*)s;

		serialiser->Write(&sensor->Intrinsic , sizeof(sensor->Intrinsic));
		serialiser->Write(&sensor->NoiseVariances, sizeof(sensor->NoiseVariances));
		serialiser->Write(&sensor->BiasVariances, sizeof(sensor->BiasVariances));

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

	bool DeserialiseSensorSpecific(const Deserialiser* d, Sensor* s) override {
		AccelerometerSensor *sensor = (AccelerometerSensor*)s;

		d->Read(&sensor->Intrinsic , sizeof(sensor->Intrinsic));
		d->Read(&sensor->NoiseVariances, sizeof(sensor->NoiseVariances));
		d->Read(&sensor->BiasVariances, sizeof(sensor->BiasVariances));

		return true;
	}
};

static slambench::io::SensorDatabaseRegistration acc_reg(AccelerometerSensor::kAccType, slambench::io::SensorDatabaseEntry(new ACCSerialiser(), new ACCDeserialiser(), false, false));


