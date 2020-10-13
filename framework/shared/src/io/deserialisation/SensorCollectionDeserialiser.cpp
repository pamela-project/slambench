/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */



#include "io/deserialisation/SensorCollectionDeserialiser.h"
#include "io/sensor/SensorCollection.h"
#include "io/sensor/SensorDatabase.h"
#include "io/sensor/Sensor.h"
#include <cstdint>

using namespace slambench::io;

SensorCollectionDeserialiser::SensorCollectionDeserialiser(std::FILE* _file) : Deserialiser(_file) {

}

bool SensorCollectionDeserialiser::Deserialise(SensorCollection& target) const {
	uint32_t sensor_count;
	Read(&sensor_count, sizeof(sensor_count));

	for(uint32_t i = 0; i < sensor_count; ++i) {
		Sensor *sensor;
		if(!DeserialiseSensor(sensor)) {
			return false;
		}

		target.AddSensor(sensor);
	}
	
	return true;
}

bool SensorCollectionDeserialiser::DeserialiseSensor(Sensor*& sensor) const {
	uint8_t sensor_name_size;
	Read(&sensor_name_size, sizeof(sensor_name_size));
	std::vector<char> namedata(sensor_name_size);
	Read(namedata.data(), sensor_name_size);

	uint8_t sensor_type_size;
	Read(&sensor_type_size, sizeof(sensor_type_size));
	std::vector<char> typedata(sensor_type_size);
	Read(typedata.data(), sensor_type_size);


	auto deserialiser = SensorDatabase::Singleton()->Get(typedata.data()).GetDeserialiser();
	return deserialiser->Deserialise(namedata.data(), typedata.data(), this, &sensor);
}
