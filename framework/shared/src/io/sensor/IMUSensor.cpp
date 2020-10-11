/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include "io/sensor/IMUSensor.h"
#include "io/sensor/SensorDatabase.h"
#include "io/serialisation/Serialiser.h"
#include "io/deserialisation/Deserialiser.h"

using namespace slambench::io;

const Sensor::sensor_type_t IMUSensor::kIMUType = "IMU";

IMUSensor::IMUSensor(const Sensor::sensor_name_t &sensor_name) : Sensor(sensor_name, kIMUType) {

}
size_t IMUSensor::GetFrameSize(const SLAMFrame *frame) const  {
	(void)frame;
	return 6 * sizeof(float);
}

class IMUSerialiser : public SensorSerialiser {
	bool SerialiseSensorSpecific(Serialiser* serialiser, const Sensor* s) override {

		IMUSensor *sensor = (IMUSensor*)s;

		serialiser->Write(&sensor->GyroscopeNoiseDensity, sizeof(sensor->GyroscopeNoiseDensity));
		serialiser->Write(&sensor->GyroscopeDriftNoiseDensity, sizeof(sensor->GyroscopeDriftNoiseDensity));
		serialiser->Write(&sensor->GyroscopeBiasDiffusion, sizeof(sensor->GyroscopeBiasDiffusion));
		serialiser->Write(&sensor->GyroscopeSaturation          , sizeof(sensor->GyroscopeSaturation));

		serialiser->Write(&sensor->AcceleratorNoiseDensity, sizeof(sensor->AcceleratorNoiseDensity));
		serialiser->Write(&sensor->AcceleratorDriftNoiseDensity , sizeof(sensor->AcceleratorDriftNoiseDensity));
		serialiser->Write(&sensor->AcceleratorBiasDiffusion, sizeof(sensor->AcceleratorBiasDiffusion));
		serialiser->Write(&sensor->AcceleratorSaturation        , sizeof(sensor->AcceleratorSaturation));



		return true;
	}
};

class IMUDeserialiser : public SensorDeserialiser {
	bool InstantiateSensor(const Sensor::sensor_name_t &sensor_name, const Sensor::sensor_type_t &type, Sensor** s) override {
		if(type != IMUSensor::kIMUType) {
			return false;
		}

		*s = new IMUSensor(sensor_name);

		return true;
	}

	bool DeserialiseSensorSpecific(const Deserialiser* d, Sensor* s) override {
		// nothing to do
		(void)d;
		(void)s;

		IMUSensor *sensor = (IMUSensor*)s;

		d->Read(&sensor->GyroscopeNoiseDensity, sizeof(sensor->GyroscopeNoiseDensity));
		d->Read(&sensor->GyroscopeDriftNoiseDensity, sizeof(sensor->GyroscopeDriftNoiseDensity));
		d->Read(&sensor->GyroscopeBiasDiffusion, sizeof(sensor->GyroscopeBiasDiffusion));
		d->Read(&sensor->GyroscopeSaturation          , sizeof(sensor->GyroscopeSaturation));

		d->Read(&sensor->AcceleratorNoiseDensity, sizeof(sensor->AcceleratorNoiseDensity));
		d->Read(&sensor->AcceleratorDriftNoiseDensity , sizeof(sensor->AcceleratorDriftNoiseDensity));
		d->Read(&sensor->AcceleratorBiasDiffusion, sizeof(sensor->AcceleratorBiasDiffusion));
		d->Read(&sensor->AcceleratorSaturation        , sizeof(sensor->AcceleratorSaturation));

		return true;
	}
};

static slambench::io::SensorDatabaseRegistration imu_reg(IMUSensor::kIMUType, slambench::io::SensorDatabaseEntry(new IMUSerialiser(), new IMUDeserialiser(), false, false));
