/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include "io/InputInterface.h"
#include "io/SLAMFile.h"
#include "io/FrameBufferSource.h"
#include "io/deserialisation/SLAMFileHeaderDeserialiser.h"
#include "io/deserialisation/SensorCollectionDeserialiser.h"

using namespace slambench::io;

InputInterface::~InputInterface() {

}

BasicInputInterface::BasicInputInterface(FrameStream& frames, SensorCollection& sensors) : frames_(frames), sensors_(sensors) {

}

FrameStream& BasicInputInterface::GetFrames() {
	return frames_;
}

SensorCollection& BasicInputInterface::GetSensors() {
	return sensors_;
}

FileInputInterface::FileInputInterface(SLAMFile& input_file) : file_(input_file), stream_(input_file) {

}

FrameStream& FileInputInterface::GetFrames() {
	return stream_;
}

SensorCollection& FileInputInterface::GetSensors() {
	return file_.Sensors;
}

FileStreamInputInterface::FileStreamInputInterface(FILE* input_file, FrameBufferSource *fb_source) : deserialiser_(input_file, sensors_, fb_source) {
	// deserialise header and sensors

	if (input_file == nullptr) {
		throw std::exception();
	}

	SLAMFileHeaderDeserialiser hdr(input_file);
	if(!hdr.Deserialise()) {
		throw std::exception();
	}
	
	SensorCollectionDeserialiser scd(input_file);
	if(!scd.Deserialise(sensors_)) {
		throw std::exception();
	}
}


FrameStream& FileStreamInputInterface::GetFrames() {
	return deserialiser_;
}

SensorCollection& FileStreamInputInterface::GetSensors() {
	return sensors_;
}
