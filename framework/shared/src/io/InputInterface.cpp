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

BasicInputInterface::BasicInputInterface(FrameStream& frames, SensorCollection& sensors) :_frames(frames), _sensors(sensors) {

}

FrameStream& BasicInputInterface::GetFrames() {
	return _frames;
}

SensorCollection& BasicInputInterface::GetSensors() {
	return _sensors;
}

FileInputInterface::FileInputInterface(SLAMFile& input_file) : _file(input_file), _stream(input_file) {

}

FrameStream& FileInputInterface::GetFrames() {
	return _stream;
}

SensorCollection& FileInputInterface::GetSensors() {
	return _file.Sensors;
}

FileStreamInputInterface::FileStreamInputInterface(FILE* input_file, FrameBufferSource *fb_source) : _deserialiser(input_file, _sensors, fb_source) {
	// deserialise header and sensors

	if (input_file == nullptr) {
		throw std::exception();
	}

	SLAMFileHeaderDeserialiser hdr(input_file);
	if(!hdr.Deserialise()) {
		throw std::exception();
	}
	
	SensorCollectionDeserialiser scd(input_file);
	if(!scd.Deserialise(_sensors)) {
		throw std::exception();
	}
}


FrameStream& FileStreamInputInterface::GetFrames() {
	return _deserialiser;
}

SensorCollection& FileStreamInputInterface::GetSensors() {
	return _sensors;
}
