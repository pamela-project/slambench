/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include "io/sensor/Sensor.h"
#include "io/serialisation/Serialiser.h"
#include "io/deserialisation/Deserialiser.h"
#include "io/sensor/SensorDatabase.h"

using namespace slambench::io;

Sensor::Sensor(const sensor_name_t &name, const sensor_type_t &type) :
		ParameterComponent(name),
		Index(0),
		Rate(0),
		sensor_name_(name),
		sensor_type_(type){
}

const Sensor::sensor_type_t &Sensor::GetType() const {
	return sensor_type_;
}

const Sensor::sensor_name_t &Sensor::GetName() const {
	return sensor_name_;
}


Sensor::~Sensor() {
}

void Sensor::CopyPose(const pose_t &other) {
	Pose = other;
}

void Sensor::CopyPose(const Sensor* other) {
	CopyPose(other->Pose);
}

bool Sensor::IsGroundTruth() const
{
	return SensorDatabase::Singleton()->Get(GetType()).IsGroundTruth();
}

bool Sensor::IsVariableSize() const
{
	return SensorDatabase::Singleton()->Get(GetType()).IsVariableSize();
}


SensorDeserialiser::~SensorDeserialiser() {

}

bool SensorDeserialiser::Deserialise(const Sensor::sensor_name_t &sensor_name, const Sensor::sensor_type_t &type, Deserialiser* d, Sensor** s) {
	Sensor *sensor;
	if(!InstantiateSensor(sensor_name, type, &sensor)) {
		fprintf(stderr, "Could not instantiate sensor of type %s\n", type.c_str());
		return false;
	}
	
	d->Read(&sensor->Index, sizeof(sensor->Index));
	uint32_t desc_bytes;
	d->Read(&desc_bytes, sizeof(desc_bytes));
	
	char desc[desc_bytes];
	d->Read(desc, desc_bytes);
	sensor->Description = desc;

	d->Read(&sensor->Rate, sizeof(sensor->Rate));
	
	d->Read(&sensor->Pose, sizeof(sensor->Pose));
	
	*s = sensor;
	bool success = DeserialiseSensorSpecific(d, *s);
	if(!success) {
		fprintf(stderr, "Sensor specific initialisation failed for '%s' (type %s)\n", desc, type.c_str());
	}
	return success;
}


SensorSerialiser::~SensorSerialiser() {

}

bool SensorSerialiser::Serialise(Serialiser* serialiser, const Sensor* sensor) {


	Sensor::sensor_type_t raw_name = sensor->GetName();
	uint8_t name_size = raw_name.size()+1;
	serialiser->Write(&name_size, sizeof(name_size));
	serialiser->Write(raw_name.c_str(), name_size);

	Sensor::sensor_type_t raw_type = sensor->GetType();
	uint8_t type_size = raw_type.size()+1;
	serialiser->Write(&type_size, sizeof(type_size));
	serialiser->Write(raw_type.c_str(), type_size);
	



	serialiser->Write(&sensor->Index, sizeof(sensor->Index));
	
	uint32_t desc_bytes = sensor->Description.size()+1;
	serialiser->Write(&desc_bytes, sizeof(desc_bytes));
	
	serialiser->Write(sensor->Description.c_str(), desc_bytes);
	serialiser->Write(&sensor->Rate, sizeof(sensor->Rate));
	serialiser->Write(&sensor->Pose, sizeof(sensor->Pose));
	
	return SerialiseSensorSpecific(serialiser, sensor);
}
