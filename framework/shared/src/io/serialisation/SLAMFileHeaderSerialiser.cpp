/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include "io/serialisation/SLAMFileHeaderSerialiser.h"
#include "io/SLAMFile.h"
#include "io/sensor/Sensor.h"
#include "io/sensor/SensorDatabase.h"

using namespace slambench::io;

SLAMFileHeaderSerialiser::SLAMFileHeaderSerialiser(FILE *file) : Serialiser(file) {
	
}

bool SLAMFileHeaderSerialiser::Serialise(const SLAMFile &file) {
	if(!SerialiseHeader(file)) return false;
	
	for(const auto &sensor : file.Sensors) {
		if(!SerialiseSensor(*sensor)) {
			printf("Could not serialise sensor %s\n", sensor->Description.c_str());
			return false;
		}
	}
	
	return true;
}

bool SLAMFileHeaderSerialiser::SerialiseHeader(const SLAMFile &file) {
	Write(SLAMFile::MagicNum, sizeof(SLAMFile::MagicNum));
	
	uint32_t version = SLAMFile::Version;
	Write(&version, sizeof(version));
	
	uint32_t sensorcount = file.Sensors.size();
	Write(&sensorcount, sizeof(sensorcount));
	
	return true;
}

bool SLAMFileHeaderSerialiser::SerialiseSensor(const Sensor &sensor) {
	if(!SensorDatabase::Singleton()->Get(sensor.GetType()).GetSerialiser()->Serialise(this, &sensor)) return false;
	
	return true;
}


