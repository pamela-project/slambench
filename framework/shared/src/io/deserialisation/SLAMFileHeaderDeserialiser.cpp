/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include "io/deserialisation/SLAMFileHeaderDeserialiser.h"
#include "io/SLAMFile.h"
#include "io/sensor/Sensor.h"
#include "io/SensorType.h"
#include "io/sensor/SensorDatabase.h"
#include <cassert>
#include "io/sensor/GroundTruthSensor.h"
#include "io/deserialisation/SensorCollectionDeserialiser.h"

using namespace slambench::io;

SLAMFileHeaderDeserialiser::SLAMFileHeaderDeserialiser(FILE *file) : Deserialiser(file) {
	
}

bool SLAMFileHeaderDeserialiser::Deserialise() {
	// check magic number
	SLAMFile::magic_num_t mnum;
	if(!Read(mnum, sizeof(mnum))) {
		fprintf(stderr, "Read failed\n");
		return false;
	}
	if(strcmp(mnum, SLAMFile::MagicNum) != 0) {
		fprintf(stderr, "SLAM file does not have correct magic number (expected %s, got %c%c%c%c)\n", SLAMFile::MagicNum, mnum[0],mnum[1],mnum[2],mnum[3]);
		return false;
	}

	SLAMFile::version_t version;
	Read(&version, sizeof(SLAMFile::version_t));
	if(version != SLAMFile::Version) {
		fprintf(stderr, "SLAM file is not correct version (expected %u, got %u)\n", SLAMFile::Version, version);
		return false;
	}

	return true;
}
