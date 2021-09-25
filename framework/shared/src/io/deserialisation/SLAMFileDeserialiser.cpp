/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include "io/deserialisation/SLAMFileDeserialiser.h"
#include "io/deserialisation/SLAMFileHeaderDeserialiser.h"
#include "io/deserialisation/SensorCollectionDeserialiser.h"

#include "io/FrameBuffer.h"
#include "io/FrameBufferSource.h"
#include "io/SLAMFile.h"
#include "io/SLAMFrame.h"
#include "io/sensor/Sensor.h"

using namespace slambench::io;

SLAMFileDeserialiser::SLAMFileDeserialiser(FILE *file, FrameBufferSource *fb_source) : Deserialiser(file), fb_source_(fb_source) {
	
}

bool SLAMFileDeserialiser::Deserialise(SLAMFile &target) const {
	if(!DeserialiseHeader(target)) return false;
	if(!DeserialiseFrames(target)) return false;
	return true;
}

bool SLAMFileDeserialiser::DeserialiseHeader(SLAMFile &target) const {
	SLAMFileHeaderDeserialiser d(File());
	if(!d.Deserialise()) {
		return false;
	}
	
	SensorCollectionDeserialiser sensor_deserialiser(File());
	if(!sensor_deserialiser.Deserialise(target.Sensors)) {
		return false;
	}
	
	return true;
}

bool SLAMFileDeserialiser::DeserialiseFrames(SLAMFile &target) const {
	SLAMFrame *frame;
	
	while(DeserialiseFrame(target, frame)) {
		target.AddFrame(frame);
	}
	
	return true;
}

bool SLAMFileDeserialiser::DeserialiseFrame(SLAMFile &file, SLAMFrame *&target) const {
	DeserialisedFrame *dsf = new DeserialisedFrame(*GetNextFramebuffer(), File());
	
	if(!Read(&dsf->Timestamp, sizeof(dsf->Timestamp))) {
		delete dsf;
		return false;
	}
	
	uint8_t sensor_index = 0;
	Read(&sensor_index, sizeof(sensor_index));
	dsf->FrameSensor = &file.Sensors.at(sensor_index);
	if(dsf->FrameSensor->IsVariableSize()) {
		uint32_t framesize;
		Read(&framesize, sizeof(framesize));
		dsf->SetVariableSize(framesize);
	}
	
	dsf->SetOffset(Offset());
	Skip(dsf->FrameSensor->GetFrameSize(dsf));
	
	target = dsf;
	
	return true;
}

FrameBuffer *SLAMFileDeserialiser::GetNextFramebuffer() const {
	return fb_source_->Next();
}
